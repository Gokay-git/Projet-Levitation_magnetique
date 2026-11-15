/**
  ******************************************************************************
  * @file    LM_ident.c
  * @author  Projet LM - ENSEA
  * @brief   Identification en boucle fermee de w0 et beta par balayage sinus.
  ******************************************************************************
  * @attention
  *
  * Pour chaque frequence : un sinus d'amplitude LM_ID_AMP_MT est ajoute a la
  * consigne de l'axe ; apres le regime transitoire, on demodule sur un nombre
  * entier de periodes la consigne r et la position p (detection synchrone).
  *   T = p / r                      reponse en boucle fermee mesuree
  *   1/G = Cr/T - Cy                procede deduit du correcteur connu
  *     Cr = b.kp + ki/s             action de la consigne
  *     Cy = kp + ki/s + kd.s/(1 + tf.s)   action de la mesure
  * Pour le modele G = beta / (s^2 - w0^2) :
  *   Re(1/G) = -(w^2 + w0^2) / beta  : droite en w^2 -> beta et w0
  *   Im(1/G) = 0                     : son ecart mesure la qualite du modele
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_ident.h"
#include "LM_log.h"
#include <math.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/** @brief Nombre complexe */
typedef struct { float re, im; } cpx_t;

/** @brief Resultat d'une frequence du balayage */
typedef struct {
  float f;       /*!< Hz */
  float t_mag;   /*!< |T| */
  float t_ph;    /*!< arg T, degres */
  float ig_re;   /*!< Re(1/G), A/mT */
  float ig_im;   /*!< Im(1/G), A/mT */
} id_pt_t;

/* Private define ------------------------------------------------------------*/
#define TWO_PI  6.28318531f

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const uint16_t id_period[LM_ID_NB_FREQ] = LM_ID_PERIODS;
static id_pt_t          pts[LM_ID_NB_FREQ];
static volatile uint8_t id_state;            /* 0 libre, 1 en cours, 2 a envoyer */
static uint32_t         id_ax, id_k, id_n, id_n_settle, id_n_end;
static float            ref_base, exc;
static float            ph_c, ph_s, rot_c, rot_s;
static float            ar_re, ar_im, ap_re, ap_im;

/* Private function prototypes -----------------------------------------------*/
static cpx_t cdiv(cpx_t a, cpx_t b);
static void  freq_start(uint32_t k);
static void  freq_end(uint32_t k);
static void  fit(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Division complexe a / b.
  * @param  a numerateur
  * @param  b denominateur
  * @retval a / b
  */
static cpx_t cdiv(cpx_t a, cpx_t b)
{
  const float d = b.re * b.re + b.im * b.im;
  cpx_t r = { (a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d };
  return r;
}

/**
  * @brief  Prepare la mesure a la frequence k (phaseur, durees, cumuls).
  * @param  k index de la frequence dans LM_ID_PERIODS
  * @retval None
  */
static void freq_start(uint32_t k)
{
  const float    per   = (float)id_period[k];
  const float    per_s = per / LM_F_LOOP_HZ;
  uint32_t       ns    = (uint32_t)ceilf(LM_ID_SETTLE_S / per_s);
  uint32_t       nm    = (uint32_t)ceilf(LM_ID_MEAS_S / per_s);

  if (ns < 1u) { ns = 1u; }
  if (nm < 2u) { nm = 2u; }
  rot_c = cosf(TWO_PI / per);
  rot_s = sinf(TWO_PI / per);
  ph_c  = 1.0f;
  ph_s  = 0.0f;
  id_n        = 0u;
  id_n_settle = ns * id_period[k];
  id_n_end    = (ns + nm) * id_period[k];
  ar_re = 0.0f; ar_im = 0.0f; ap_re = 0.0f; ap_im = 0.0f;
}

/**
  * @brief  Termine la frequence k : calcule T et 1/G.
  * @param  k index de la frequence dans LM_ID_PERIODS
  * @retval None
  */
static void freq_end(uint32_t k)
{
  const lm_pid_t *P = lm_axis_pid(id_ax);
  const float     w = TWO_PI * LM_F_LOOP_HZ / (float)id_period[k];
  const cpx_t     ar = { ar_re, ar_im }, ap = { ap_re, ap_im };
  const cpx_t     T  = cdiv(ap, ar);
  const cpx_t     Cr = { P->b * P->kp, -P->ki / w };
  const cpx_t     jw = { 0.0f, P->kd * w };
  const cpx_t     lp = { 1.0f, w * P->tf_d };
  const cpx_t     dv = cdiv(jw, lp);
  const cpx_t     Cy = { P->kp + dv.re, -P->ki / w + dv.im };
  const cpx_t     q  = cdiv(Cr, T);

  pts[k].f     = LM_F_LOOP_HZ / (float)id_period[k];
  pts[k].t_mag = sqrtf(T.re * T.re + T.im * T.im);
  pts[k].t_ph  = atan2f(T.im, T.re) * (360.0f / TWO_PI);
  pts[k].ig_re = q.re - Cy.re;
  pts[k].ig_im = q.im - Cy.im;
}

/**
  * @brief  Ajuste w0 et beta sur les points mesures.
  * @note   Moindres carres en erreur relative (poids 1/y^2, pour que les
  *         basses frequences, qui fixent w0, comptent autant que les hautes) :
  *         Re(1/G) = a.w^2 + c, avec a = -1/beta et c = a.w0^2.
  * @retval None
  */
static void fit(void)
{
  float n = 0.0f, sw = 0.0f, sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, q = 0.0f;

  for (uint32_t k = 0; k < LM_ID_NB_FREQ; k++) {
    const float w = TWO_PI * pts[k].f, x = w * w, y = pts[k].ig_re;
    const float p = 1.0f / fmaxf(y * y, 1e-12f);
    n += 1.0f; sw += p; sx += p * x; sy += p * y; sxx += p * x * x; sxy += p * x * y;
    q += fabsf(pts[k].ig_im) / fmaxf(fabsf(y), 1e-12f);
  }
  const float den = sw * sxx - sx * sx;
  lm.id_ok[id_ax] = 0u;
  if (fabsf(den) < 1e-30f) {
    return;
  }
  const float a    = (sw * sxy - sx * sy) / den;
  const float c    = (sy - a * sx) / sw;
  const float w0sq = (a != 0.0f) ? (c / a) : 0.0f;

  lm.id_beta[id_ax]    = (a != 0.0f) ? (-1.0f / a) : 0.0f;
  lm.id_w0[id_ax]      = (w0sq >= 0.0f) ? sqrtf(w0sq) : -sqrtf(-w0sq);
  lm.id_quality[id_ax] = q / n;
  lm.id_ok[id_ax]      = (uint8_t)((a < 0.0f) && (w0sq > 0.0f));
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Lance un balayage sur un axe.
  * @note   Accepte seulement en levitation et si aucun balayage n'est en cours.
  * @param  axis axe (LM_AXIS_X ou LM_AXIS_Y)
  * @retval None
  */
void ident_request(uint32_t axis)
{
  if ((id_state != 0u) || (axis >= LM_NB_AXIS) || (lm.mode != LM_MODE_LEVITATION)) {
    return;
  }
  id_ax    = axis;
  ref_base = lm_axis_pid(axis)->ref;
  id_k     = 0u;
  exc      = 0.0f;
  freq_start(0u);
  id_state = 1u;
}

/**
  * @brief  Interrompt le balayage et restaure la consigne.
  * @retval None
  */
void ident_abort(void)
{
  if (id_state == 1u) {
    lm_axis_pid(id_ax)->ref = ref_base;
    id_state = 0u;
  }
}

/**
  * @brief  Indique si un balayage est en cours.
  * @retval 1 pendant le balayage, 0 sinon
  */
int ident_running(void)
{
  return id_state == 1u;
}

/**
  * @brief  Ajoute le sinus a la consigne (avant ctrl_position()).
  * @retval None
  */
void ident_pre(void)
{
  if (id_state != 1u) {
    return;
  }
  exc = LM_ID_AMP_MT * ph_s;
  lm_axis_pid(id_ax)->ref = ref_base + exc;
}

/**
  * @brief  Demodule la consigne et la position (apres ctrl_position()).
  * @retval None
  */
void ident_post(void)
{
  if (id_state != 1u) {
    return;
  }
  const float p = (id_ax == (uint32_t)LM_AXIS_X) ? lm.px : lm.py;

  if (id_n >= id_n_settle) {
    ar_re += exc * ph_c;  ar_im -= exc * ph_s;
    ap_re += p * ph_c;    ap_im -= p * ph_s;
  }
  /* rotation du phaseur, renormalise a chaque pas */
  const float c = ph_c * rot_c - ph_s * rot_s;
  const float s = ph_s * rot_c + ph_c * rot_s;
  const float g = 1.5f - 0.5f * (c * c + s * s);
  ph_c = c * g;
  ph_s = s * g;

  if (++id_n >= id_n_end) {
    freq_end(id_k);
    if (++id_k < LM_ID_NB_FREQ) {
      freq_start(id_k);
    } else {
      lm_axis_pid(id_ax)->ref = ref_base;
      fit();
      id_state = 2u;
    }
  }
}

/**
  * @brief  Envoie les resultats du balayage (boucle principale).
  * @note   #BODE,axe puis f_Hz,|T|,phase_T_deg,Re(1/G),Im(1/G) (1/G en mA/mT),
  *         puis #ID,axe,w0 (rad/s),beta (mT/(s2.A)),qualite (0 = parfait),valide
  * @retval None
  */
void ident_send(void)
{
  char line[128], a[16], b[16], c[16], d[16], e[16];

  if (id_state != 2u) {
    return;
  }
  log_write(line, snprintf(line, sizeof(line), "#BODE,%lu\r\n", (unsigned long)id_ax));
  for (uint32_t k = 0; k < LM_ID_NB_FREQ; k++) {
    log_ftoa(a, sizeof(a), pts[k].f, 2);
    log_ftoa(b, sizeof(b), pts[k].t_mag, 3);
    log_ftoa(c, sizeof(c), pts[k].t_ph, 1);
    log_ftoa(d, sizeof(d), pts[k].ig_re * 1000.0f, 3);
    log_ftoa(e, sizeof(e), pts[k].ig_im * 1000.0f, 3);
    log_write(line, snprintf(line, sizeof(line), "%s,%s,%s,%s,%s\r\n", a, b, c, d, e));
  }
  log_ftoa(a, sizeof(a), lm.id_w0[id_ax], 2);
  log_ftoa(b, sizeof(b), lm.id_beta[id_ax], 0);
  log_ftoa(c, sizeof(c), lm.id_quality[id_ax], 3);
  log_write(line, snprintf(line, sizeof(line), "#ID,%lu,%s,%s,%s,%u\r\n",
                           (unsigned long)id_ax, a, b, c, (unsigned)lm.id_ok[id_ax]));
  log_write(line, snprintf(line, sizeof(line), "#END\r\n"));
  id_state = 0u;
}
