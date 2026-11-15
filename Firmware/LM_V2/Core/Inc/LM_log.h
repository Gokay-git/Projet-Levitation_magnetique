/**
  ******************************************************************************
  * @file    LM_log.h
  * @author  Projet LM - ENSEA
  * @brief   Log serie a 500 Hz, essai echelon enregistre a 5 kHz, rapport de
  *          reglage.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_LOG_H
#define __LM_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void log_capture_request(void);
void log_capture_abort(void);
void log_capture_update(void);
void log_snapshot(uint32_t mode);
void log_config_request(void);
void log_send(void);

/* utilitaires partages (boucle principale uniquement) */
void log_write(const char *s, int n);
int  log_ftoa(char *dst, int size, float v, int dec);

#ifdef __cplusplus
}
#endif

#endif /* __LM_LOG_H */
