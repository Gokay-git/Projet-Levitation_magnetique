/**
  ******************************************************************************
  * @file    LM_safety.c
  * @author  Projet LM - ENSEA
  * @brief   Securites : surintensite, tension de bus, objet perdu.
  ******************************************************************************
  * @attention
  *
  * Un defaut n'est pris en compte qu'apres plusieurs echantillons consecutifs,
  * pour ne pas couper sur un seul point bruite.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_safety.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t oc_cnt[LM_NB_COIL];
static uint32_t vlow_cnt, vhigh_cnt;
static uint32_t lost_cnt;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Remet a zero les compteurs de defaut.
  * @retval None
  */
void safety_reset(void)
{
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    oc_cnt[k] = 0u;
  }
  vlow_cnt  = 0u;
  vhigh_cnt = 0u;
  lost_cnt  = 0u;
}

/**
  * @brief  Surveille les courants et la tension de bus ; ecrit lm.fault.
  * @retval None
  */
void safety_check(void)
{
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    oc_cnt[k] = (fabsf(lm.i[k]) > LM_I_TRIP_A) ? (oc_cnt[k] + 1u) : 0u;
    if (oc_cnt[k] >= LM_I_TRIP_SAMPLES) {
      lm.fault = LM_FAULT_OVERCURRENT;
    }
  }

  vlow_cnt  = (lm.vbus < LM_VBUS_MIN_V) ? (vlow_cnt + 1u)  : 0u;
  vhigh_cnt = (lm.vbus > LM_VBUS_MAX_V) ? (vhigh_cnt + 1u) : 0u;
  if (vlow_cnt >= LM_VBUS_SAMPLES) {
    lm.fault = LM_FAULT_VBUS_LOW;
  }
  if (vhigh_cnt >= LM_VBUS_SAMPLES) {
    lm.fault = LM_FAULT_VBUS_HIGH;
  }
}

/**
  * @brief  Detecte la perte de l'objet en levitation.
  * @retval 1 si l'erreur de position depasse lm.e_lost pendant lm.lost_ms
  */
int safety_object_lost(void)
{
  const float ex = lm.ax.ref - lm.px;
  const float ey = lm.ay.ref - lm.py;
  const uint32_t lost_ticks = (uint32_t)((float)lm.lost_ms * LM_F_LOOP_HZ / 1000.0f);

  lost_cnt = ((fabsf(ex) > lm.e_lost) || (fabsf(ey) > lm.e_lost)) ? (lost_cnt + 1u) : 0u;
  return lost_cnt >= lost_ticks;
}
