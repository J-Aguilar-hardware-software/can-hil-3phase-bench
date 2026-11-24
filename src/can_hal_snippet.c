/**
 * @file    can_hal_snippet.c
 * @brief   Blocs d'extension matérielle DCANA (500 kbps) à intégrer dans hal.c.
 * @details Destiné au projet TI C2000Ware MotorControl SDK — Universal Motor
 * Control Lab (Build Level 3) sur carte LAUNCHXL-F2800137 + BOOSTXL-DRV8323RS
 * (Site 1 : J1/J3/J4/J2).
 *
 * @copyright Copyright (c) 2026 Jorge Aguilar - Extensions CAN Protocol.
 *            Distribué sous licence Apache License 2.0 (voir fichier LICENSE).
 */

/* ========================================================================== */
/* BLOC 1 : À COLLER DANS hal.c -> HAL_setParams() (vers la ligne 347)        */
/*          Juste après HAL_setupI2CA(handle);                                */
/* ========================================================================== */

/* ========================================================================== */
/* EXTENSION PROJET CAN — Copyright (c) 2026 Jorge Aguilar (Licence Apache 2.0)
 */
/* ========================================================================== */
HAL_setupCAN(handle);
/* ========================================================================== */
/* END EXTENSION PROJET CAN                                                   */
/* ========================================================================== */

/* ========================================================================== */
/* BLOC 1 : À COLLER DANS hal.c -> HAL_setupGPIOs() (vers la ligne 1909)      */
/*          Remplace la configuration par défaut des broches réservées        */
/*          GPIO12 et GPIO13 pour le routage physique CANA_RX / CANA_TX.      */
/* ========================================================================== */

/* ========================================================================== */
/* EXTENSION PROJET CAN — Copyright (c) 2026 Jorge Aguilar (Licence Apache 2.0)
 */
/* ========================================================================== */
/* Configuration de GPIO13 en tant que CANA_TX (Sortie) vers transceiver 3,3 V
 */
GPIO_setPinConfig(GPIO_13_CANA_TX);
GPIO_setDirectionMode(13U, GPIO_DIR_MODE_OUT);
GPIO_setPadConfig(13U, GPIO_PIN_TYPE_STD);

/* Configuration de GPIO12 en tant que CANA_RX (Entrée) depuis transceiver 3,3 V
 */
GPIO_setPinConfig(GPIO_12_CANA_RX);
GPIO_setDirectionMode(12U, GPIO_DIR_MODE_IN);
GPIO_setPadConfig(12U, GPIO_PIN_TYPE_STD);

/* Mode asynchrone requis pour l'échantillonnage matériel DCAN à 500 kbps */
GPIO_setQualificationMode(12U, GPIO_QUAL_ASYNC);
/* ========================================================================== */
/* END EXTENSION PROJET CAN                                                   */
/* ========================================================================== */

/* ========================================================================== */
/* BLOC 2 : À COLLER EN FIN DE hal.c (implémentation de HAL_setupCAN)         */
/*          Appeler ensuite HAL_setupCAN(halHandle); dans main() (sys_main.c) */
/*          juste après HAL_setParams(halHandle, &userParams);                */
/* ========================================================================== */

/* ========================================================================== */
/* EXTENSION PROJET CAN — Copyright (c) 2026 Jorge Aguilar (Licence Apache 2.0)
 */
/* ========================================================================== */
/**
 * @brief   Initialise et configure le contrôleur matériel DCANA (500 kbps) et
 * ses boîtes aux lettres.
 *
 * @details Active l'horloge du périphérique CANA sur le TMS320F2800137,
 * configure le débit binaire à 500 kbps (20 TQ à 120 MHz) et alloue les 4
 * objets de message (Mailboxes) en mode scrutation (polling) sans interruption
 * matérielle imbriquée :
 *          - Mailbox 1 (TX) : ID 0x101 — Canal Moyenne Priorité (télémétrie
 * temps réel)
 *          - Mailbox 2 (TX) : ID 0x102 — Canal Basse Priorité (énumération
 * réseau & statuts)
 *          - Mailbox 5 (TX) : ID 0x100 — Canal Haute Priorité (alarmes moteur &
 * erreurs protocole)
 *          - Mailbox 3 (RX) : ID 0x555 — Canal de réception avec filtre
 * matériel exact (0x7FF)
 *
 * @param[in] handle  Handle d'abstraction matérielle HAL (non utilisé dans
 * cette implémentation).
 */
void HAL_setupCAN(HAL_Handle handle) {
  /* Paramètre non utilisé dans cette implémentation (MISRA C:2012 Rule 2.7) */
  (void)handle;

  /* 1. Activation de l'horloge physique du module CANA */
  SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);

  /* 2. Sélection de l'horloge système (SYSCLK) et initialisation */
  CAN_selectClockSource(CANA_BASE, CAN_CLOCK_SOURCE_SYS);
  CAN_initModule(CANA_BASE);

  /* 3. Configuration du débit à 500 kbps (20 TQ à 120 MHz) */
  CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000UL, 20U);

  /* 4. Mailboxes TX (Target Node C2000 -> CAN Server STM32) */
  /* Mailbox 1 : Canal Priorité Moyenne (CAN_MP - 0x101) */
  CAN_setupMessageObject(CANA_BASE, 1U, 0x101UL, CAN_MSG_FRAME_STD,
                         CAN_MSG_OBJ_TYPE_TX, 0UL, CAN_MSG_OBJ_NO_FLAGS, 8U);

  /* Mailbox 2 : Canal Basse Priorité (CAN_LP - 0x102) */
  CAN_setupMessageObject(CANA_BASE, 2U, 0x102UL, CAN_MSG_FRAME_STD,
                         CAN_MSG_OBJ_TYPE_TX, 0UL, CAN_MSG_OBJ_NO_FLAGS, 8U);

  /* Mailbox 5 : Canal Haute Priorité (CAN_HP - 0x100 : Alarmes & Sécurité) */
  CAN_setupMessageObject(CANA_BASE, 5U, 0x100UL, CAN_MSG_FRAME_STD,
                         CAN_MSG_OBJ_TYPE_TX, 0UL, CAN_MSG_OBJ_NO_FLAGS, 8U);

  /* 5. Mailbox RX (CAN Server STM32 -> Target Node C2000 sur ID exact 0x555,
   * polling) */
  CAN_setupMessageObject(CANA_BASE, 3U, 0x555UL, CAN_MSG_FRAME_STD,
                         CAN_MSG_OBJ_TYPE_RX, 0x7FFUL,
                         CAN_MSG_OBJ_USE_ID_FILTER, 8U);

  /* 6. Démarrage physique du contrôleur CAN */
  CAN_startModule(CANA_BASE);
}
/* ========================================================================== */
/* END EXTENSION PROJET CAN                                                   */
/* ========================================================================== */
