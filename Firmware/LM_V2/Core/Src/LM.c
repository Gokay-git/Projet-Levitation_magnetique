/**
  ******************************************************************************
  * @file    LM.c
  * @author  Projet LM - ENSEA
  * @brief   Sequencement de la boucle de levitation, interruptions et
  *          demarrage.
  ******************************************************************************
  * @attention
  *
  * TIM1 (PWM centree 5 kHz) declenche l'ADC1 a chaque periode, le DMA range
  * les 8 mesures et lm_step() tourne dans HAL_ADC_ConvCpltCallback().
  *
  * Commandes (lm.cmd, ecrit depuis le debugger, Live Expressions) :
  *   1 calibration des zeros      2 capture de la consigne     3 RAZ integrales
  *   4 echelon sur step_coil      5 effacer le defaut
  *   6 balayage d'identification sur l'axe id_axis (en levitation)
  *   7 appliquer w0 et beta identifies sur id_axis   8 recalculer les gains
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_measure.h"
#include "LM_control.h"
#include "LM_bridge.h"
#include "LM_safety.h"
#include "LM_log.h"
#include "LM_ident.h"
#include <string.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;

/* Exported variables --------------------------------------------------------*/
lm_t lm;

/* Private variables ---------------------------------------------------------*/
static volatile uint16_t adc_buf[LM_NB_ADC];
static lm_mode_t         prev_mode = LM_MODE_OFF;

/* Private function prototypes -----------------------------------------------*/
static int       mode_is_active(lm_mode_t m);
static void      handle_command(void);
static lm_mode_t resolve_mode(void);
static void      lm_step(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Indique si un mode commande les ponts.
  * @param  m mode
  * @retval 1 en tension, courant ou levitation, 0 sinon
  */
static int mode_is_active(lm_mode_t m)
{
  return (m == LM_MODE_VOLTAGE) || (m == LM_MODE_CURRENT) || (m == LM_MODE_LEVITATION);
}

/**
  * @brief  Execute la commande ecrite dans lm.cmd depuis le debugger.
  * @retval None
  */
static void handle_command(void)
{
  const lm_cmd_t cmd = lm.cmd;
  if (cmd == LM_CMD_NONE) {
    return;
  }
  lm.cmd = LM_CMD_NONE;

  switch (cmd) {
    case LM_CMD_CALIB_ZERO:
      meas_calib_start();
      lm.mode = LM_MODE_OFF;
      break;
    case LM_CMD_CAPTURE_REF:
      ctrl_capture_ref();
      break;
    case LM_CMD_RESET_INTEGRAL:
      ctrl_reset_integrals();
      break;
    case LM_CMD_STEP:
      log_capture_request();
      break;
    case LM_CMD_CLEAR_FAULT:
      lm.fault = LM_FAULT_NONE;
      lm.mode  = LM_MODE_OFF;
      break;
    case LM_CMD_SWEEP:
      ident_request(lm.id_axis);
      break;
    case LM_CMD_APPLY_ID:
      if ((lm.id_axis < LM_NB_AXIS) && (lm.id_ok[lm.id_axis] != 0u)) {
        lm.w0[lm.id_axis]   = lm.id_w0[lm.id_axis];
        lm.beta[lm.id_axis] = lm.id_beta[lm.id_axis];
        ctrl_retune();
        log_config_request();
      }
      break;
    case LM_CMD_RETUNE:
      ctrl_retune();
      log_config_request();
      break;
    default:
      break;
  }
}

/**
  * @brief  Mode effectif apres calibration et securites.
  * @note   Au changement de mode, les etats des correcteurs sont remis a zero
  *         pour eviter les a-coups.
  * @retval mode a appliquer pendant cette periode
  */
static lm_mode_t resolve_mode(void)
{
  lm_mode_t m = lm.mode;

  if (meas_calib_running() || ((uint32_t)m > (uint32_t)LM_MODE_FAULT)) {
    m = LM_MODE_OFF;
    lm.mode = m;
  }
  if ((m == LM_MODE_LEVITATION) && (lm.cfg_ok == 0u) && (lm.fault == LM_FAULT_NONE)) {
    lm.fault = LM_FAULT_CONFIG;   /* reglage incoherent : voir #CFG */
  }
  if ((lm.fault == LM_FAULT_NONE) && mode_is_active(m)) {
    safety_check();
  }
  if (lm.fault != LM_FAULT_NONE) {
    m = LM_MODE_FAULT;
    lm.mode = m;
  }
  if (m != prev_mode) {
    ident_abort();
    ctrl_reset();
    safety_reset();
    log_capture_abort();
    prev_mode = m;
  }
  return m;
}

/**
  * @brief  Une periode de la boucle a 5 kHz : mesures, correcteurs, ponts, log.
  * @retval None
  */
static void lm_step(void)
{
  float sd[LM_NB_COIL] = { 0.0f, 0.0f, 0.0f, 0.0f };

  meas_update(adc_buf);
  handle_command();
  lm_mode_t m = resolve_mode();

  switch (m) {
    case LM_MODE_VOLTAGE:
      for (uint32_t k = 0; k < LM_NB_COIL; k++) {
        const float v = lm.v_manual[k];
        lm.i_ref[k] = 0.0f;
        lm.v_cmd[k] = v;
        sd[k] = (fabsf(v) < 0.01f) ? 0.0f : bridge_duty(v, (v > 0.0f) ? 1.0f : -1.0f, lm.vbus);
      }
      break;

    case LM_MODE_CURRENT:
      for (uint32_t k = 0; k < LM_NB_COIL; k++) {
        lm.i_ref[k] = lm_clampf(lm.i_manual[k], -lm.i_lim, lm.i_lim);
      }
      break;

    case LM_MODE_LEVITATION:
      if (safety_object_lost()) {
        lm.fault = LM_FAULT_OBJECT_LOST;
        lm.mode  = LM_MODE_FAULT;
        m        = LM_MODE_FAULT;
      } else {
        ident_pre();
        ctrl_position();
        ident_post();
      }
      break;

    default:
      break;
  }

  /* boucles de courant */
  if ((m == LM_MODE_CURRENT) || (m == LM_MODE_LEVITATION)) {
    for (uint32_t k = 0; k < LM_NB_COIL; k++) {
      const float ir = lm.i_ref[k];
      if (ir == 0.0f) {          /* EN = 0 : le courant s'eteint seul */
        ctrl_current_reset(k);
        lm.v_cmd[k] = 0.0f;
        sd[k]       = 0.0f;
      } else {
        lm.v_cmd[k] = ctrl_current(k);
        sd[k]       = bridge_duty(lm.v_cmd[k], (ir > 0.0f) ? 1.0f : -1.0f, lm.vbus);
      }
    }
  }

  if (mode_is_active(m)) {
    for (uint32_t k = 0; k < LM_NB_COIL; k++) {
      lm.duty[k] = sd[k];
      bridge_apply(k, sd[k]);
    }
  } else {
    for (uint32_t k = 0; k < LM_NB_COIL; k++) {
      lm.duty[k] = 0.0f;
    }
    bridge_all_off();
  }

  log_capture_update();
  log_snapshot((uint32_t)m);
  lm.loop_count++;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Fin de conversion ADC (DMA) : lance une periode de la boucle.
  * @note   La sortie TIMING est a 1 pendant le calcul (mesure a l'oscilloscope).
  * @param  hadc ADC concerne
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) {
    return;
  }
  TIMING_GPIO_Port->BSRR = TIMING_Pin;
  lm_step();
  TIMING_GPIO_Port->BSRR = (uint32_t)TIMING_Pin << 16;
}

/**
  * @brief  Bouton B1 : marche / arret, efface aussi un defaut.
  * @param  GPIO_Pin broche ayant declenche l'interruption
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last = 0u;

  if (GPIO_Pin != B1_Pin) {
    return;
  }
  const uint32_t t = HAL_GetTick();
  if ((t - last) < 250u) {
    return;
  }
  last = t;

  if ((lm.mode == LM_MODE_OFF) && (lm.fault == LM_FAULT_NONE)) {
    lm.mode = LM_MODE_LEVITATION;
  } else {
    lm.fault = LM_FAULT_NONE;
    lm.mode  = LM_MODE_OFF;
  }
}

/**
  * @brief  Initialise la structure lm et verifie la configuration CubeMX.
  * @note   A appeler apres les MX_xxx_Init(). Appelle Error_Handler() si la
  *         configuration ne correspond pas a LM_config.h.
  * @retval None
  */
void LM_Init(void)
{
  static const float mix0[2][LM_NB_HALL]  = LM_MIX_INIT;
  static const float sign0[LM_NB_COIL]    = LM_COIL_SIGN_INIT;

  memset(&lm, 0, sizeof(lm));
  ctrl_init();
  memcpy(lm.mix, mix0, sizeof(lm.mix));
  memcpy(lm.coil_sign, sign0, sizeof(lm.coil_sign));
  lm.step_value = 0.2f;
  lm.e_lost     = LM_E_LOST_MT;
  lm.lost_ms    = LM_LOST_MS;
  log_config_request();   /* lignes #CFG envoyees au demarrage */

  /* verification de la configuration CubeMX */
  if (!bridge_ports_ok()) {
    Error_Handler();
  }
  if (hadc1.Init.NbrOfConversion != (uint32_t)LM_NB_ADC) {
    Error_Handler();
  }
  const float fpwm = (float)HAL_RCC_GetPCLK2Freq() /
                     ((float)(htim1.Instance->PSC + 1u) * 2.0f * (float)__HAL_TIM_GET_AUTORELOAD(&htim1));
  if (((htim1.Instance->CR1 & TIM_CR1_CMS) == 0u) || (htim1.Instance->RCR != 1u) ||
      (fabsf(fpwm - LM_F_LOOP_HZ) > 0.01f * LM_F_LOOP_HZ)) {
    Error_Handler();
  }
}

/**
  * @brief  Demarre l'ADC (DMA) puis les 4 PWM de TIM1, ponts coupes.
  * @note   Une calibration des zeros est lancee : l'objet doit etre retire.
  * @retval None
  */
void LM_Start(void)
{
  bridge_all_off();

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, LM_NB_ADC) != HAL_OK) {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);

  lm.cmd = LM_CMD_CALIB_ZERO;

  if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)) {
    Error_Handler();
  }
}

/**
  * @brief  Taches lentes de la boucle principale : envoi du log, LED LD2.
  * @note   LD2 : fixe en levitation, 1 Hz en essai, 5 Hz en defaut.
  * @retval None
  */
void LM_Background(void)
{
  log_send();
  ident_send();

  const uint32_t t = HAL_GetTick();
  GPIO_PinState led;
  switch (lm.mode) {
    case LM_MODE_LEVITATION: led = GPIO_PIN_SET;                                      break;
    case LM_MODE_VOLTAGE:
    case LM_MODE_CURRENT:    led = ((t / 500u) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET; break;
    case LM_MODE_FAULT:      led = ((t / 100u) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET; break;
    default:                 led = GPIO_PIN_RESET;                                    break;
  }
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, led);
}
