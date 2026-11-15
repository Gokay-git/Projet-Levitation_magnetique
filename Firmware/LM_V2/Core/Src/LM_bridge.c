/**
  ******************************************************************************
  * @file    LM_bridge.c
  * @author  Projet LM - ENSEA
  * @brief   Commande des 4 demi-ponts L293D : sens sur INa/INb, rapport
  *          cyclique sur EN (TIM1 CH1 a CH4).
  ******************************************************************************
  * @attention
  *
  * Quand EN = 0 les sorties du L293D sont en haute impedance : le courant se
  * decharge par les diodes internes et la bobine voit -Vbus. En conduction
  * continue : V = s(2d - 1)Vbus, donc d = (1 + sV/Vbus)/2.
  * C'est le seul fichier a reecrire si on change de pont.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LM.h"
#include "LM_bridge.h"

/* Private typedef -----------------------------------------------------------*/
/** @brief Broches et voie PWM d'un pont */
typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin_a;
  uint16_t      pin_b;
  uint32_t      tim_ch;
} bridge_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* Private variables ---------------------------------------------------------*/
static const bridge_t bridge[LM_NB_COIL] = {
  [LM_COIL_X1] = { X1_A_GPIO_Port, X1_A_Pin, X1_B_Pin, TIM_CHANNEL_1 },
  [LM_COIL_X2] = { X2_A_GPIO_Port, X2_A_Pin, X2_B_Pin, TIM_CHANNEL_2 },
  [LM_COIL_Y1] = { Y1_A_GPIO_Port, Y1_A_Pin, Y1_B_Pin, TIM_CHANNEL_3 },
  [LM_COIL_Y2] = { Y2_A_GPIO_Port, Y2_A_Pin, Y2_B_Pin, TIM_CHANNEL_4 },
};

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Tension voulue -> rapport cyclique signe.
  * @param  v    tension voulue aux bornes de la bobine (V)
  * @param  s    sens du courant (+1 ou -1)
  * @param  vbus tension de bus mesuree (V)
  * @retval rapport cyclique signe : signe = sens, valeur absolue dans [0 ; 1]
  */
float bridge_duty(float v, float s, float vbus)
{
  if (vbus < 1.0f) {
    return 0.0f;
  }
  return s * lm_clampf(0.5f * (1.0f + s * v / vbus), 0.0f, 1.0f);
}

/**
  * @brief  Applique un rapport cyclique signe a un pont.
  * @note   INa et INb changent dans la meme ecriture BSRR.
  * @param  k  bobine (LM_COIL_X1 a LM_COIL_Y2)
  * @param  sd signe = sens, valeur absolue = rapport cyclique sur EN
  * @retval None
  */
void bridge_apply(uint32_t k, float sd)
{
  const bridge_t *br  = &bridge[k];
  const float     arr = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
  float mag;

  if (sd > 0.0f) {
    br->port->BSRR = (uint32_t)br->pin_a | ((uint32_t)br->pin_b << 16);
    mag = sd;
  } else if (sd < 0.0f) {
    br->port->BSRR = (uint32_t)br->pin_b | ((uint32_t)br->pin_a << 16);
    mag = -sd;
  } else {
    br->port->BSRR = ((uint32_t)br->pin_a << 16) | ((uint32_t)br->pin_b << 16);
    mag = 0.0f;
  }
  __HAL_TIM_SET_COMPARE(&htim1, br->tim_ch, (uint32_t)(mag * arr + 0.5f));
}

/**
  * @brief  Coupe les 4 ponts (EN = 0, INa = INb = 0).
  * @retval None
  */
void bridge_all_off(void)
{
  for (uint32_t k = 0; k < LM_NB_COIL; k++) {
    bridge_apply(k, 0.0f);
  }
}

/**
  * @brief  Verifie que INa et INb d'un meme pont sont sur le meme port.
  * @retval 1 si la configuration CubeMX est correcte, 0 sinon
  */
int bridge_ports_ok(void)
{
  return (X1_A_GPIO_Port == X1_B_GPIO_Port) && (X2_A_GPIO_Port == X2_B_GPIO_Port) &&
         (Y1_A_GPIO_Port == Y1_B_GPIO_Port) && (Y2_A_GPIO_Port == Y2_B_GPIO_Port);
}
