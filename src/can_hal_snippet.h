/**
 * @file    can_hal_snippet.h
 * @brief   Prototype HAL d'initialisation DCANA à coller dans hal.h (vers la
 * ligne 3468).
 * @details Destiné au projet TI C2000Ware MotorControl SDK — Universal Motor
 * Control Lab (Build Level 3) sur carte LAUNCHXL-F2800137 + BOOSTXL-DRV8323RS
 * (Site 1 : J1/J3/J4/J2).
 *
 * @copyright Copyright (c) 2026 Jorge Aguilar - Extensions CAN Protocol.
 *            Distribué sous licence Apache License 2.0 (voir fichier LICENSE).
 */

/* ========================================================================== */
/* À COLLER DANS hal.h (vers la ligne 3468, fin des prototypes HAL)           */
/* ========================================================================== */

/* ========================================================================== */
/* EXTENSION PROJET CAN — Copyright (c) 2026 Jorge Aguilar (Licence Apache 2.0)
 */
/* ========================================================================== */
/**
 * @brief      Configure le périphérique matériel DCANA (500 kbps, Mailboxes
 * TX/RX).
 * @param[in]  handle  Handle de la structure d'abstraction matérielle HAL.
 */
void HAL_setupCAN(HAL_Handle handle);
/* ========================================================================== */
/* END EXTENSION PROJET CAN                                                   */
/* ========================================================================== */
