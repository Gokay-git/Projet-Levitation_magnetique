/**
  ******************************************************************************
  * @file    LM_measure.h
  * @author  Projet LM - ENSEA
  * @brief   Conversion des mesures ADC, calibration des zeros, calcul de la
  *          position.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_MEASURE_H
#define __LM_MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void meas_update(const volatile uint16_t *adc);
void meas_calib_start(void);
int  meas_calib_running(void);

#ifdef __cplusplus
}
#endif

#endif /* __LM_MEASURE_H */
