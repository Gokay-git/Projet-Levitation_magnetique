/**
  ******************************************************************************
  * @file    LM_bridge.h
  * @author  Projet LM - ENSEA
  * @brief   Commande des ponts L293D : sens sur INa/INb, rapport cyclique sur EN.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_BRIDGE_H
#define __LM_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
float bridge_duty(float v, float s, float vbus);
void  bridge_apply(uint32_t k, float sd);
void  bridge_all_off(void);
int   bridge_ports_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* __LM_BRIDGE_H */
