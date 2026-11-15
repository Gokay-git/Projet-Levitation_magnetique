/**
  ******************************************************************************
  * @file    LM_measure.c
  * @author  Projet LM - ENSEA
  * @brief   Conversion des mesures ADC en grandeurs physiques, calibration des
  *          zeros et calcul de la position X/Y.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_measure.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ADC_MID_LSB   2047.5f
#define V_PER_LSB     (LM_VREF_V / LM_ADC_FS)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t calib_left;
static float    calib_b[LM_NB_HALL];
static float    calib_i[LM_NB_COIL];

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Lance la calibration des zeros (moyenne sur LM_CALIB_SAMPLES).
  * @note   Bobines coupees et objet retire pendant la calibration.
  * @retval None
  */
void meas_calib_start(void)
{
  calib_left = LM_CALIB_SAMPLES;
  memset(calib_b, 0, sizeof(calib_b));
  memset(calib_i, 0, sizeof(calib_i));
}

/**
  * @brief  Indique si la calibration est en cours.
  * @retval 1 pendant la calibration, 0 sinon
  */
int meas_calib_running(void)
{
  return calib_left > 0u;
}

/**
  * @brief  Convertit les 8 mesures ADC et met a jour lm.i, lm.b, lm.vbus,
  *         lm.px et lm.py.
  * @param  adc tableau des 8 mesures, dans l'ordre des Ranks CubeMX
  * @retval None
  */
void meas_update(const volatile uint16_t *adc)
{
  float i_raw[LM_NB_COIL];
  float b_raw[LM_NB_HALL];

  /* courants : 0 A au milieu de l'echelle */
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    i_raw[k] = ((float)adc[LM_ADC_I_X1 + k] - ADC_MID_LSB) * V_PER_LSB / LM_I_V_PER_A;
    lm.i[k]  = i_raw[k] - lm.i0[k];
  }

  /* capteurs Hall : 0 mT au milieu de l'echelle (DRV5055 ratiometrique) */
  for (uint32_t h = 0; h < LM_NB_HALL; h++) {
    b_raw[h] = ((float)adc[LM_ADC_HALL1 + h] - ADC_MID_LSB) * V_PER_LSB / LM_HALL_V_PER_MT;
    lm.b[h]  = b_raw[h] - lm.b0[h];
  }

  lm.vbus = (float)adc[LM_ADC_VBUS] * V_PER_LSB * LM_VBUS_RATIO;

  /* calibration : moyenne des valeurs brutes, bobines coupees */
  if (calib_left > 0u) {
    for (uint32_t h = 0; h < LM_NB_HALL; h++) {
      calib_b[h] += b_raw[h];
    }
    for (uint32_t k = 0; k < LM_NB_COIL; k++) {
      calib_i[k] += i_raw[k];
    }
    if (--calib_left == 0u) {
      for (uint32_t h = 0; h < LM_NB_HALL; h++) {
        lm.b0[h] = calib_b[h] / (float)LM_CALIB_SAMPLES;
      }
      for (uint32_t k = 0; k < LM_NB_COIL; k++) {
        lm.i0[k] = calib_i[k] / (float)LM_CALIB_SAMPLES;
      }
    }
  }

  /* on retire le champ cree par les bobines, puis position X/Y */
  for (uint32_t h = 0; h < LM_NB_HALL; h++) {
    float bc = lm.b[h];
    for (uint32_t k = 0; k < LM_NB_COIL; k++) {
      bc -= lm.g_ff[h][k] * lm.i[k];
    }
    lm.b_corr[h] = bc;
  }

  float px = 0.0f, py = 0.0f;
  for (uint32_t h = 0; h < LM_NB_HALL; h++) {
    px += lm.mix[0][h] * lm.b_corr[h];
    py += lm.mix[1][h] * lm.b_corr[h];
  }
  lm.px = px;
  lm.py = py;
}
