/**
  ******************************************************************************
  * @file    LM_ident.h
  * @author  Projet LM - ENSEA
  * @brief   Identification en boucle fermee : balayage sinus sur la consigne
  *          d'un axe, mesure de la reponse en frequence, puis ajustement du
  *          modele p'' = w0^2.p + beta.u.
  ******************************************************************************
  * @attention
  *
  * Le systeme est instable : on ne peut pas ouvrir la boucle pour le mesurer.
  * On l'identifie donc en levitation, a partir du correcteur connu.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_IDENT_H
#define __LM_IDENT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void ident_request(uint32_t axis);
void ident_abort(void);
int  ident_running(void);
void ident_pre(void);
void ident_post(void);
void ident_send(void);

#ifdef __cplusplus
}
#endif

#endif /* __LM_IDENT_H */
