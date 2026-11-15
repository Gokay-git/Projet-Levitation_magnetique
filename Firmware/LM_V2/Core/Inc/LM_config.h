/**
  ******************************************************************************
  * @file    LM_config.h
  * @author  Projet LM - ENSEA
  * @brief   Tous les reglages du systeme de levitation magnetique.
  ******************************************************************************
  * @attention
  *
  * Deux natures de reglages :
  *  - "a mesurer" : grandeur physique, a remplacer par la valeur mesuree ;
  *  - "choix"     : choix de conception, a garder sauf raison precise.
  * C'est le seul fichier a modifier pour adapter le firmware a la maquette.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM_CONFIG_H
#define __LM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @defgroup LM_Config_Coils Bobines (a mesurer : mode 1, essai echelon, tau = L/R)
  * @{
  */
#define LM_COIL_R_OHM       10.0f
#define LM_COIL_L_H         0.020f
#define LM_FC_CURRENT_HZ    200.0f   /*!< choix : bande passante de la boucle de courant */
/**
  * @}
  */

/** @defgroup LM_Config_Model Modele lateral de chaque axe (a mesurer : commande 6, balayage)
  * @brief  p'' = w0^2 . p + beta . u
  *         p en mT (combinaison des capteurs Hall), u en A (commande d'axe).
  *         Valeurs provisoires : w0 suppose, beta choisi pour retrouver
  *         kp = 0,05 A/mT. Remplacer par les valeurs identifiees.
  * @{
  */
#define LM_W0_X_RAD_S       30.0f    /*!< a mesurer : pulsation d'instabilite, rad/s */
#define LM_BETA_X           90000.0f /*!< a mesurer : mT/(s^2.A) */
#define LM_W0_Y_RAD_S       30.0f
#define LM_BETA_Y           90000.0f
/**
  * @}
  */

/** @defgroup LM_Config_Position Correcteur de position : choix de conception
  * @brief  Ordres de grandeur valides sur le banc virtuel Sys Lin.
  * @{
  */
#define LM_POS_ZETA         0.7f     /*!< amortissement vise */
#define LM_POS_WN_RATIO     2.0f     /*!< wn = ratio x w0 */
#define LM_POS_TI_RATIO     5.0f     /*!< Ti = ratio x Td (4 a 7 : meilleur compromis) */
#define LM_POS_TF_RATIO     8.0f     /*!< filtre de la derivee : tf = Td / ratio (5 a 10) */
#define LM_POS_B            0.5f     /*!< ponderation de consigne : 1 = PID classique */
#define LM_ROUTH_MARGIN     5.0f     /*!< Td.Ti.wn^2 >= marge (instable sous 1) */
#define LM_CUR_POS_RATIO    6.0f     /*!< boucle de courant >= ratio x wn */
#define LM_POS_I_MAX        0.2f     /*!< borne du terme integral, A */
#define LM_I_LIM            0.5f     /*!< consigne de courant max, A */
/**
  * @brief  Courant de biais en levitation (choix).
  *         Chaque paire X1/X2, Y1/Y2 porte un courant de repos dont les forces
  *         laterales s'annulent. Le courant ne passe plus par 0 : le L293D reste
  *         en conduction continue et la boucle de courant reste lineaire
  *         (simulation : gain 0,82 -> 0,99 a 25 Hz). A prendre superieur aux
  *         corrections usuelles plus la moitie de l'ondulation. 0 pour desactiver.
  *         Le champ du biais vu par les capteurs Hall se compense par g_ff.
  */
#define LM_I_BIAS_A         0.15f
/**
  * @}
  */

/** @defgroup LM_Config_Geometry Geometrie (a adapter)
  * @{
  */
/** Position des capteurs : px = H1 - H3, py = H2 - H3 */
#define LM_MIX_INIT         { { 1.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, -1.0f } }
/** Sens de bobinage X1 X2 Y1 Y2 (a verifier : beta doit sortir positif) */
#define LM_COIL_SIGN_INIT   { +1.0f, -1.0f, +1.0f, -1.0f }
/**
  * @}
  */

/** @defgroup LM_Config_Ident Identification en boucle fermee (commande 6)
  * @{
  */
#define LM_ID_AMP_MT        2.0f     /*!< amplitude du sinus ajoute a la consigne */
#define LM_ID_NB_FREQ       7u
/** periode de chaque sinus en echantillons a 5 kHz : 2, 3, 5, 8, 12, 18, 25 Hz */
#define LM_ID_PERIODS       { 2500u, 1667u, 1000u, 625u, 417u, 278u, 200u }
#define LM_ID_SETTLE_S      0.3f     /*!< attente du regime etabli */
#define LM_ID_MEAS_S        0.5f     /*!< duree de mesure (nombre entier de periodes) */
/**
  * @}
  */

/** @defgroup LM_Config_Measure Chaine de mesure (schema V1)
  * @{
  */
#define LM_VREF_V           3.3f
#define LM_ADC_FS           4095.0f
#define LM_HALL_V_PER_MT    0.0075f  /*!< DRV5055A4 sous 3,3 V */
#define LM_I_V_PER_A        2.0f     /*!< shunt 0,1 ohm x INA240A1 (gain 20) */
#define LM_VBUS_RATIO       7.8f     /*!< (68k + 10k) / 10k */
/**
  * @}
  */

/** @defgroup LM_Config_Safety Securites
  * @{
  */
#define LM_I_TRIP_A         0.75f
#define LM_I_TRIP_SAMPLES   5u       /*!< 1 ms */
#define LM_VBUS_MIN_V       8.0f
#define LM_VBUS_MAX_V       20.0f    /*!< l'ADC sature a 3,3 x 7,8 = 25,7 V */
#define LM_VBUS_SAMPLES     10u      /*!< 2 ms */
#define LM_E_LOST_MT        40.0f
#define LM_LOST_MS          100u
/**
  * @}
  */

/** @defgroup LM_Config_Timing Cadencement
  * @{
  */
#define LM_F_LOOP_HZ        5000.0f
#define LM_TS               (1.0f / LM_F_LOOP_HZ)
#define LM_CALIB_SAMPLES    2500u    /*!< 0,5 s */
#define LM_LOG_DIV          10u      /*!< log a 500 Hz */
#define LM_CAP_LEN          512u     /*!< essai echelon : 102 ms */
#define LM_CAP_PRE          64u
/**
  * @}
  */

/** @defgroup LM_Config_ADC Ordre des voies ADC = ordre des Ranks dans CubeMX
  * @{
  */
enum {
  LM_ADC_I_X1 = 0,   /*!< IN15 PB0 */
  LM_ADC_I_X2,       /*!< IN2  PC1 */
  LM_ADC_I_Y1,       /*!< IN1  PC0 */
  LM_ADC_I_Y2,       /*!< IN3  PC2 */
  LM_ADC_HALL1,      /*!< IN5  PA0 */
  LM_ADC_HALL2,      /*!< IN6  PA1 */
  LM_ADC_HALL3,      /*!< IN9  PA4 */
  LM_ADC_VBUS,       /*!< IN4  PC3 */
  LM_NB_ADC
};

#define LM_NB_HALL          3u
#define LM_NB_COIL          4u
#define LM_NB_AXIS          2u
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __LM_CONFIG_H */
