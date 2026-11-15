/**
  ******************************************************************************
  * @file    LM_control.h
  * @author  Projet LM - ENSEA
  * @brief   PID de position (un par axe) et PI de courant (un par bobine).
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_CONTROL_H
#define __LM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void  ctrl_init(void);
void  ctrl_retune(void);
void  ctrl_reset(void);
void  ctrl_reset_integrals(void);
void  ctrl_capture_ref(void);
void  ctrl_position(void);
float ctrl_current(uint32_t k);
void  ctrl_current_reset(uint32_t k);

#ifdef __cplusplus
}
#endif

#endif /* __LM_CONTROL_H */
