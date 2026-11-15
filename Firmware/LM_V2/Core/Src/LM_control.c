/**
  ******************************************************************************
  * @file    LM_control.c
  * @author  Projet LM - ENSEA
  * @brief   PID de position (un par axe) et PI de courant (un par bobine).
  ******************************************************************************
  * @attention
  *
  * Boucle de position, par axe : p'' = w0^2.p + beta.u (modele lateral)
  *   PID par placement de poles : on vise s^2 + 2.zeta.wn.s + wn^2
  *     kp = (wn^2 + w0^2) / beta     kd = 2.zeta.wn / beta
  *   Integrale : Ti = ratio x Td, relevee si besoin pour garder
  *     Td.Ti.wn^2 >= marge (critere de Routh, instable sous 1)
  *   Derivee sur la mesure, filtree (tf = Td / ratio)
  *   Ponderation de consigne b sur le terme proportionnel : supprime
  *   le gros depassement du PID sur un changement de consigne sans
  *   modifier le rejet des perturbations.
  *
  * Boucle de courant, par bobine : PI qui compense le pole R/L.
  * En levitation, un courant de biais a forces opposees est ajoute a chaque
  * paire de bobines pour eviter la conduction discontinue du L293D autour de 0 A.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_control.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/** @brief Etat interne d'un PID de position */
typedef struct {
  float integ;
  float d_filt;
  float p_prev;
} pid_state_t;

/* Private define ------------------------------------------------------------*/
#define TWO_PI  6.28318531f

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static pid_state_t sx, sy;
static float       pi_integ[LM_NB_COIL];

/* Private function prototypes -----------------------------------------------*/
static void  pid_reset(pid_state_t *s, float meas);
static float pid_update(const lm_pid_t *P, pid_state_t *S, float meas, float u_max);
static int   design_axis(uint32_t a);
static void  pair_refs(uint32_t k1, uint32_t k2, float u);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Remet a zero l'etat d'un PID.
  * @param  s    etat du PID
  * @param  meas mesure actuelle (evite un pic de derivee)
  * @retval None
  */
static void pid_reset(pid_state_t *s, float meas)
{
  s->integ  = 0.0f;
  s->d_filt = 0.0f;
  s->p_prev = meas;
}

/**
  * @brief  Une periode du PID de position.
  * @note   Integrale sur l'erreur complete, proportionnel sur la consigne
  *         ponderee, derivee filtree sur la mesure, anti-emballement.
  * @param  P     gains et consigne
  * @param  S     etat du PID
  * @param  meas  position mesuree (mT)
  * @param  u_max sortie maximale (A)
  * @retval commande d'axe (A)
  */
static float pid_update(const lm_pid_t *P, pid_state_t *S, float meas, float u_max)
{
  const float e     = P->ref - meas;                                 /* erreur complete : integrale */
  const float e_p   = (P->ref0 + P->b * (P->ref - P->ref0)) - meas;   /* consigne ponderee : proportionnel */
  const float d_raw = (meas - S->p_prev) * LM_F_LOOP_HZ;
  S->p_prev = meas;

  const float a = LM_TS / (P->tf_d + LM_TS);
  S->d_filt += a * (d_raw - S->d_filt);

  const float u_pd  = P->kp * e_p - P->kd * S->d_filt;
  const float u_tst = u_pd + S->integ;

  if ((fabsf(u_tst) < u_max) || ((u_tst * e) < 0.0f)) {
    S->integ += P->ki * e * LM_TS;
    S->integ  = lm_clampf(S->integ, -P->i_max, P->i_max);
  }
  return lm_clampf(u_pd + S->integ, -u_max, u_max);
}

/**
  * @brief  Calcule les gains d'un axe a partir de son modele (w0, beta).
  * @param  a axe (LM_AXIS_X ou LM_AXIS_Y)
  * @retval 1 si le reglage est coherent (Routh et marge de la boucle de
  *         courant respectes), 0 sinon
  */
static int design_axis(uint32_t a)
{
  lm_pid_t   *P    = lm_axis_pid(a);
  const float w0   = lm.w0[a];
  const float beta = lm.beta[a];
  const float wn   = LM_POS_WN_RATIO * w0;
  const float wc   = TWO_PI * LM_FC_CURRENT_HZ;

  P->b     = LM_POS_B;
  P->i_max = LM_POS_I_MAX;
  lm.wn[a] = wn;
  lm.cur_ratio[a] = (wn > 0.0f) ? (wc / wn) : 0.0f;

  if (!(w0 > 0.0f) || !(beta > 0.0f)) {
    P->kp = 0.0f;
    P->ki = 0.0f;
    P->kd = 0.0f;
    P->tf_d = 0.0f;
    lm.routh[a] = 0.0f;
    return 0;
  }

  P->kp = (wn * wn + w0 * w0) / beta;
  P->kd = 2.0f * LM_POS_ZETA * wn / beta;

  const float td     = P->kd / P->kp;
  const float ti_min = LM_ROUTH_MARGIN / (td * wn * wn);
  float       ti     = LM_POS_TI_RATIO * td;
  if (ti < ti_min) {
    ti = ti_min;
  }
  P->ki   = P->kp / ti;
  P->tf_d = td / LM_POS_TF_RATIO;
  lm.routh[a] = td * ti * wn * wn;

  return (lm.routh[a] >= 1.0f) && (lm.cur_ratio[a] >= LM_CUR_POS_RATIO);
}

/**
  * @brief  Consignes de courant d'une paire de bobines.
  * @note   Commande d'axe en push-pull + biais a forces opposees.
  *         Force laterale ~ s1.i1 + s2.i2 : avec b1 = +B et b2 = -B.s1.s2,
  *         s1.b1 + s2.b2 = 0.
  * @param  k1 premiere bobine de la paire
  * @param  k2 seconde bobine de la paire
  * @param  u  commande d'axe (A)
  * @retval None
  */
static void pair_refs(uint32_t k1, uint32_t k2, float u)
{
  const float s1 = lm.coil_sign[k1], s2 = lm.coil_sign[k2];
  const float b1 = lm.i_bias, b2 = -lm.i_bias * s1 * s2;
  lm.i_ref[k1] = lm_clampf(s1 * u + b1, -lm.i_lim, lm.i_lim);
  lm.i_ref[k2] = lm_clampf(s2 * u + b2, -lm.i_lim, lm.i_lim);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Recalcule les gains des deux axes depuis lm.w0[] et lm.beta[],
  *         puis met a jour lm.cfg_ok.
  * @retval None
  */
void ctrl_retune(void)
{
  const int okx = design_axis(LM_AXIS_X);
  const int oky = design_axis(LM_AXIS_Y);
  lm.cfg_ok = (uint8_t)(okx && oky);
  sx.integ = 0.0f;
  sy.integ = 0.0f;
}

/**
  * @brief  Gains initiaux depuis LM_config.h.
  * @note   PI de courant : compensation du pole R/L, kp = L.wc, ki = R.wc.
  * @retval None
  */
void ctrl_init(void)
{
  const float wc = TWO_PI * LM_FC_CURRENT_HZ;
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    lm.cur[k].kp = LM_COIL_L_H * wc;
    lm.cur[k].ki = LM_COIL_R_OHM * wc;
  }

  lm.w0[LM_AXIS_X]   = LM_W0_X_RAD_S;
  lm.beta[LM_AXIS_X] = LM_BETA_X;
  lm.w0[LM_AXIS_Y]   = LM_W0_Y_RAD_S;
  lm.beta[LM_AXIS_Y] = LM_BETA_Y;
  lm.ax.ref  = 0.0f;
  lm.ax.ref0 = 0.0f;
  lm.ay.ref  = 0.0f;
  lm.ay.ref0 = 0.0f;
  lm.i_lim   = LM_I_LIM;
  lm.i_bias  = LM_I_BIAS_A;
  ctrl_retune();
}

/**
  * @brief  Remet a zero tous les correcteurs et les consignes.
  * @retval None
  */
void ctrl_reset(void)
{
  pid_reset(&sx, lm.px);
  pid_reset(&sy, lm.py);
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    pi_integ[k] = 0.0f;
    lm.i_ref[k] = 0.0f;
    lm.v_cmd[k] = 0.0f;
  }
  lm.ux = 0.0f;
  lm.uy = 0.0f;
}

/**
  * @brief  Remet a zero les termes integraux (position et courant).
  * @retval None
  */
void ctrl_reset_integrals(void)
{
  sx.integ = 0.0f;
  sy.integ = 0.0f;
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    pi_integ[k] = 0.0f;
  }
}

/**
  * @brief  La position actuelle devient la consigne et le point de
  *         fonctionnement.
  * @retval None
  */
void ctrl_capture_ref(void)
{
  lm.ax.ref  = lm.px;
  lm.ax.ref0 = lm.px;
  lm.ay.ref  = lm.py;
  lm.ay.ref0 = lm.py;
  pid_reset(&sx, lm.px);
  pid_reset(&sy, lm.py);
}

/**
  * @brief  Boucle de position : position -> consignes de courant des 4 bobines.
  * @retval None
  */
void ctrl_position(void)
{
  const float u_max = lm.i_lim - fabsf(lm.i_bias);   /* marge laissee au biais */

  lm.ux = pid_update(&lm.ax, &sx, lm.px, u_max);
  lm.uy = pid_update(&lm.ay, &sy, lm.py, u_max);
  pair_refs(LM_COIL_X1, LM_COIL_X2, lm.ux);
  pair_refs(LM_COIL_Y1, LM_COIL_Y2, lm.uy);
}

/**
  * @brief  PI de courant d'une bobine.
  * @param  k bobine (LM_COIL_X1 a LM_COIL_Y2)
  * @retval tension a appliquer (V), limitee a +/- Vbus
  */
float ctrl_current(uint32_t k)
{
  const float v_lim = lm.vbus;
  const float e     = lm.i_ref[k] - lm.i[k];
  const float v_u   = lm.cur[k].kp * e + pi_integ[k];

  if ((fabsf(v_u) < v_lim) || ((v_u * e) < 0.0f)) {
    pi_integ[k] += lm.cur[k].ki * e * LM_TS;
    pi_integ[k]  = lm_clampf(pi_integ[k], -v_lim, v_lim);
  }
  return lm_clampf(lm.cur[k].kp * e + pi_integ[k], -v_lim, v_lim);
}

/**
  * @brief  Remet a zero le terme integral du PI de courant d'une bobine.
  * @param  k bobine (LM_COIL_X1 a LM_COIL_Y2)
  * @retval None
  */
void ctrl_current_reset(uint32_t k)
{
  pi_integ[k] = 0.0f;
}
