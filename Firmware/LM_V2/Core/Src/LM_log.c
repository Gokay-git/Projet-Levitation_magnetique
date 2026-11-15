/**
  ******************************************************************************
  * @file    LM_log.c
  * @author  Projet LM - ENSEA
  * @brief   Log serie (USART2, 921600 bauds) : donnees a 500 Hz, essai
  *          echelon enregistre a 5 kHz, rapport de reglage.
  ******************************************************************************
  * @attention
  *
  * Les donnees sont preparees dans l'interruption et envoyees dans la boucle
  * principale : l'UART est trop lent pour l'interruption.
  * Lignes commencant par # : capture (#CAP), reglage (#CFG),
  * identification (#BODE, #ID).
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_log.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/** @brief Instantane envoye sur une ligne de log */
typedef struct {
  uint32_t n;
  int32_t  i[LM_NB_COIL];   /*!< mA */
  int32_t  px, py;          /*!< 0,01 mT */
  int32_t  vbus;            /*!< mV */
  uint32_t mode;
  uint32_t fault;
} log_snap_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Private variables ---------------------------------------------------------*/
static log_snap_t       log_snap;
static volatile uint8_t log_ready;
static uint32_t         log_div;

static int16_t          cap_ref[LM_CAP_LEN];
static int16_t          cap_i[LM_CAP_LEN];
static uint32_t         cap_idx;
static uint32_t         cap_coil;
static lm_mode_t        cap_mode;
static volatile uint8_t cap_state;   /* 0 libre, 1 en cours, 2 a envoyer */
static volatile uint8_t cfg_pending;

/* Private function prototypes -----------------------------------------------*/
static inline int16_t to_i16(float v);
static void           send_config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Conversion saturee en entier 16 bits.
  * @param  v valeur
  * @retval valeur bornee a +/- 32767
  */
static inline int16_t to_i16(float v)
{
  return (int16_t)lm_clampf(v, -32767.0f, 32767.0f);
}

/**
  * @brief  Envoie une ligne #CFG par axe.
  * @note   #CFG,axe,kp (A/mT),ki (A/(mT.s)),kd (mA.s/mT),b,tf (ms),
  *         wn (rad/s),Routh,courant/wn,ok
  * @retval None
  */
static void send_config(void)
{
  char line[128], f[8][16];

  for (uint32_t a = 0; a < LM_NB_AXIS; a++) {
    const lm_pid_t *P = lm_axis_pid(a);
    log_ftoa(f[0], 16, P->kp, 3);
    log_ftoa(f[1], 16, P->ki, 3);
    log_ftoa(f[2], 16, P->kd * 1000.0f, 3);
    log_ftoa(f[3], 16, P->b, 2);
    log_ftoa(f[4], 16, P->tf_d * 1000.0f, 2);
    log_ftoa(f[5], 16, lm.wn[a], 1);
    log_ftoa(f[6], 16, lm.routh[a], 2);
    log_ftoa(f[7], 16, lm.cur_ratio[a], 1);
    log_write(line, snprintf(line, sizeof(line), "#CFG,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%u\r\n",
                             (unsigned long)a, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
                             (unsigned)lm.cfg_ok));
  }
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Demande un essai echelon sur la bobine lm.step_coil.
  * @note   Accepte seulement en mode tension ou courant.
  * @retval None
  */
void log_capture_request(void)
{
  if (((lm.mode == LM_MODE_VOLTAGE) || (lm.mode == LM_MODE_CURRENT)) &&
      (lm.step_coil < LM_NB_COIL) && (cap_state == 0u)) {
    cap_coil  = lm.step_coil;
    cap_mode  = lm.mode;
    cap_idx   = 0u;
    cap_state = 1u;
  }
}

/**
  * @brief  Annule un essai echelon en cours.
  * @retval None
  */
void log_capture_abort(void)
{
  if (cap_state == 1u) {
    cap_state = 0u;
  }
}

/**
  * @brief  Enregistre un point de l'essai echelon (appele a 5 kHz).
  * @note   L'echelon est applique au point LM_CAP_PRE.
  * @retval None
  */
void log_capture_update(void)
{
  if (cap_state != 1u) {
    return;
  }
  const float ref = (cap_mode == LM_MODE_VOLTAGE) ? lm.v_manual[cap_coil] : lm.i_ref[cap_coil];
  cap_ref[cap_idx] = to_i16(ref * 1000.0f);
  cap_i[cap_idx]   = to_i16(lm.i[cap_coil] * 1000.0f);

  if (cap_idx == LM_CAP_PRE) {
    if (cap_mode == LM_MODE_VOLTAGE) {
      lm.v_manual[cap_coil] = lm.step_value;
    } else {
      lm.i_manual[cap_coil] = lm.step_value;
    }
  }
  if (++cap_idx >= LM_CAP_LEN) {
    cap_state = 2u;
  }
}

/**
  * @brief  Prepare une ligne de log tous les LM_LOG_DIV appels (500 Hz).
  * @param  mode mode effectif de la periode
  * @retval None
  */
void log_snapshot(uint32_t mode)
{
  if ((++log_div < LM_LOG_DIV) || (log_ready != 0u)) {
    return;
  }
  log_div = 0u;
  log_snap.n = lm.loop_count;
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    log_snap.i[k] = (int32_t)(lm.i[k] * 1000.0f);
  }
  log_snap.px    = (int32_t)(lm.px * 100.0f);
  log_snap.py    = (int32_t)(lm.py * 100.0f);
  log_snap.vbus  = (int32_t)(lm.vbus * 1000.0f);
  log_snap.mode  = mode;
  log_snap.fault = (uint32_t)lm.fault;
  log_ready = 1u;
}

/**
  * @brief  Envoie n octets sur l'UART (boucle principale uniquement).
  * @param  s texte
  * @param  n nombre d'octets (ignore si hors de ]0 ; 128[)
  * @retval None
  */
void log_write(const char *s, int n)
{
  if ((n > 0) && (n < 128)) {
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)n, 5u);
  }
}

/**
  * @brief  Ecrit v avec dec decimales, sans le printf flottant de newlib-nano.
  * @param  dst  tampon de sortie
  * @param  size taille du tampon
  * @param  v    valeur
  * @param  dec  nombre de decimales (0 a 3)
  * @retval nombre de caracteres ecrits (comme snprintf)
  */
int log_ftoa(char *dst, int size, float v, int dec)
{
  static const long p10[4] = { 1L, 10L, 100L, 1000L };
  if (dec < 0) { dec = 0; }
  if (dec > 3) { dec = 3; }
  if (!(v == v)) {
    return snprintf(dst, (size_t)size, "nan");
  }
  if (v > 2.0e9f / (float)p10[dec]) { v = 2.0e9f / (float)p10[dec]; }
  if (v < -2.0e9f / (float)p10[dec]) { v = -2.0e9f / (float)p10[dec]; }
  const int  neg = v < 0.0f;
  const long q   = (long)((neg ? -v : v) * (float)p10[dec] + 0.5f);
  const long ip  = q / p10[dec], fp = q % p10[dec];
  switch (dec) {
    case 0:  return snprintf(dst, (size_t)size, "%s%ld", neg ? "-" : "", ip);
    case 1:  return snprintf(dst, (size_t)size, "%s%ld.%01ld", neg ? "-" : "", ip, fp);
    case 2:  return snprintf(dst, (size_t)size, "%s%ld.%02ld", neg ? "-" : "", ip, fp);
    default: return snprintf(dst, (size_t)size, "%s%ld.%03ld", neg ? "-" : "", ip, fp);
  }
}

/**
  * @brief  Demande l'envoi des lignes #CFG au prochain log_send().
  * @retval None
  */
void log_config_request(void)
{
  cfg_pending = 1u;
}

/**
  * @brief  Envoie ce qui est pret : #CFG, essai echelon (#CAP) ou ligne de
  *         donnees (boucle principale).
  * @retval None
  */
void log_send(void)
{
  char line[128];

  if (cfg_pending != 0u) {
    cfg_pending = 0u;
    send_config();
  }

  if (cap_state == 2u) {
    /* #CAP,bobine,mode puis k,consigne,courant (mV ou mA, mA) */
    log_write(line, snprintf(line, sizeof(line), "#CAP,%lu,%lu\r\n",
                             (unsigned long)cap_coil, (unsigned long)cap_mode));
    for (uint32_t j = 0; j < LM_CAP_LEN; j++) {
      log_write(line, snprintf(line, sizeof(line), "%lu,%d,%d\r\n",
                               (unsigned long)j, (int)cap_ref[j], (int)cap_i[j]));
    }
    log_write(line, snprintf(line, sizeof(line), "#END\r\n"));
    cap_state = 0u;
    return;
  }

  if (log_ready != 0u) {
    /* n, iX1, iX2, iY1, iY2 (mA), px, py (0,01 mT), vbus (mV), mode, defaut */
    log_snap_t s;
    __disable_irq();
    s = log_snap;
    log_ready = 0u;
    __enable_irq();

    log_write(line, snprintf(line, sizeof(line), "%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu\r\n",
                             (unsigned long)s.n,
                             (long)s.i[0], (long)s.i[1], (long)s.i[2], (long)s.i[3],
                             (long)s.px, (long)s.py, (long)s.vbus,
                             (unsigned long)s.mode, (unsigned long)s.fault));
  }
}
