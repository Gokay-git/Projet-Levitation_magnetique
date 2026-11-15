/**
  ******************************************************************************
  * @file    LM.h
  * @author  Projet LM - ENSEA
  * @brief   Levitation magnetique 2 axes : types, structure globale lm et
  *          fonctions appelees par main.c.
  ******************************************************************************
  * @attention
  *
  * Toutes les grandeurs du systeme sont regroupees dans la structure lm, pour
  * etre lues et modifiees en direct depuis le debugger (Live Expressions).
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_H
#define __LM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "LM_config.h"

/* Exported types ------------------------------------------------------------*/

/** @brief Index des bobines */
typedef enum { LM_COIL_X1 = 0, LM_COIL_X2 = 1, LM_COIL_Y1 = 2, LM_COIL_Y2 = 3 } lm_coil_t;

/** @brief Index des axes */
typedef enum { LM_AXIS_X = 0, LM_AXIS_Y = 1 } lm_axis_t;

/** @brief Modes de fonctionnement (lm.mode) */
typedef enum {
  LM_MODE_OFF        = 0,
  LM_MODE_VOLTAGE    = 1,  /*!< tension imposee v_manual[] */
  LM_MODE_CURRENT    = 2,  /*!< courant impose i_manual[] */
  LM_MODE_LEVITATION = 3,
  LM_MODE_FAULT      = 4
} lm_mode_t;

/** @brief Commandes (lm.cmd, ecrites depuis le debugger) */
typedef enum {
  LM_CMD_NONE           = 0,
  LM_CMD_CALIB_ZERO     = 1,
  LM_CMD_CAPTURE_REF    = 2,
  LM_CMD_RESET_INTEGRAL = 3,
  LM_CMD_STEP           = 4,  /*!< essai echelon sur la bobine step_coil */
  LM_CMD_CLEAR_FAULT    = 5,
  LM_CMD_SWEEP          = 6,  /*!< balayage sinus en levitation sur l'axe id_axis */
  LM_CMD_APPLY_ID       = 7,  /*!< reprend w0 et beta identifies sur id_axis */
  LM_CMD_RETUNE         = 8   /*!< recalcule les gains depuis w0[] et beta[] */
} lm_cmd_t;

/** @brief Defauts (lm.fault) */
typedef enum {
  LM_FAULT_NONE        = 0,
  LM_FAULT_OVERCURRENT = 1,
  LM_FAULT_VBUS_LOW    = 2,
  LM_FAULT_VBUS_HIGH   = 3,
  LM_FAULT_OBJECT_LOST = 4,
  LM_FAULT_CONFIG      = 5   /*!< reglage incoherent (voir la ligne #CFG du log) */
} lm_fault_t;

/** @brief Gains d'un PI de courant */
typedef struct {
  float kp;     /*!< V/A */
  float ki;     /*!< V/(A.s) */
} lm_pi_t;

/** @brief Gains et consigne d'un PID de position */
typedef struct {
  float kp;     /*!< A/mT */
  float ki;     /*!< A/(mT.s) */
  float kd;     /*!< A.s/mT */
  float tf_d;   /*!< s */
  float b;      /*!< ponderation de consigne sur le terme proportionnel */
  float i_max;  /*!< A */
  float ref;    /*!< mT, consigne */
  float ref0;   /*!< mT, point de fonctionnement (consigne capturee) */
} lm_pid_t;

/** @brief Etat complet du systeme */
typedef struct {
  /* reglages */
  volatile lm_mode_t mode;
  volatile lm_cmd_t  cmd;
  lm_pid_t ax, ay;
  lm_pi_t  cur[LM_NB_COIL];
  float i_lim;
  float i_bias;                         /*!< A, courant de biais en levitation */
  float coil_sign[LM_NB_COIL];
  float mix[2][LM_NB_HALL];
  float g_ff[LM_NB_HALL][LM_NB_COIL];   /*!< mT/A */
  float v_manual[LM_NB_COIL];
  float i_manual[LM_NB_COIL];
  uint32_t step_coil;
  float step_value;
  float e_lost;
  uint32_t lost_ms;

  /* modele de chaque axe et diagnostic du reglage */
  float w0[LM_NB_AXIS];         /*!< rad/s */
  float beta[LM_NB_AXIS];       /*!< mT/(s^2.A) */
  float wn[LM_NB_AXIS];         /*!< rad/s, pulsation visee */
  float routh[LM_NB_AXIS];      /*!< Td.Ti.wn^2, stable si > 1 */
  float cur_ratio[LM_NB_AXIS];  /*!< bande passante courant / wn */
  uint8_t cfg_ok;               /*!< 0 : levitation refusee */

  /* identification */
  uint32_t id_axis;
  float id_w0[LM_NB_AXIS];
  float id_beta[LM_NB_AXIS];
  float id_quality[LM_NB_AXIS]; /*!< |Im| / |Re| moyen de 1/G, ideal 0 */
  uint8_t id_ok[LM_NB_AXIS];

  /* mesures */
  volatile lm_fault_t fault;
  float i0[LM_NB_COIL];
  float b0[LM_NB_HALL];
  float i[LM_NB_COIL];
  float vbus;
  float b[LM_NB_HALL];
  float b_corr[LM_NB_HALL];
  float px, py;
  float ux, uy;
  float i_ref[LM_NB_COIL];
  float v_cmd[LM_NB_COIL];
  float duty[LM_NB_COIL];
  uint32_t loop_count;
} lm_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern lm_t lm;

/* Exported functions (inline) -----------------------------------------------*/

/**
  * @brief  Borne une valeur.
  * @param  v  valeur
  * @param  lo borne basse
  * @param  hi borne haute
  * @retval valeur bornee
  */
static inline float lm_clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/**
  * @brief  PID de position d'un axe.
  * @param  a axe (LM_AXIS_X ou LM_AXIS_Y)
  * @retval pointeur vers lm.ax ou lm.ay
  */
static inline lm_pid_t *lm_axis_pid(uint32_t a)
{
  return (a == (uint32_t)LM_AXIS_X) ? &lm.ax : &lm.ay;
}

/* Exported functions prototypes ---------------------------------------------*/
void LM_Init(void);
void LM_Start(void);
void LM_Background(void);

#ifdef __cplusplus
}
#endif

#endif /* __LM_H */
