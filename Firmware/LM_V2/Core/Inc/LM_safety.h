/**
  ******************************************************************************
  * @file    LM_safety.h
  * @author  Projet LM - ENSEA
  * @brief   Securites : surintensite, tension de bus, objet perdu.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_SAFETY_H
#define __LM_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void safety_reset(void);
void safety_check(void);
int  safety_object_lost(void);

#ifdef __cplusplus
}
#endif

#endif /* __LM_SAFETY_H */
