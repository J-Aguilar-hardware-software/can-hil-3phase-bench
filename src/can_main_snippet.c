/**
 * @file    can_main_snippet.c
 * @brief   Blocs d'initialisation et de tâche cyclique CAN 2.0A (1 ms RX / 10
 * ms TX) à coller dans sys_main.c.
 * @details Destiné au projet TI C2000Ware MotorControl SDK — Universal Motor
 * Control Lab (Build Level 3) sur carte LAUNCHXL-F2800137 + BOOSTXL-DRV8323RS
 * (Site 1 : J1/J3/J4/J2).
 *
 * @copyright Copyright (c) 2026 Jorge Aguilar - Extensions CAN Protocol.
 *            Distribué sous licence Apache License 2.0 (voir fichier LICENSE).
 */

/* ========================================================================== */
/* BLOC 1 : À COLLER DANS LA BOUCLE PRINCIPALE DE sys_main.c                  */
/*          À l'intérieur de la base de temps 1 ms (HAL_CPU_TIMER0),          */
/*          juste après HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);     */
/* ========================================================================== */

/* ========================================================================== */
/* EXTENSION PROJET CAN — Copyright (c) 2026 Jorge Aguilar (Licence Apache 2.0)
 */
/* ========================================================================== */

/* VARIABLES */
static uint16_t tx_timer_cnt   = 0U;
static uint8_t  tx_counter_hp  = 0U; /* Compteur cyclique canal 0x100 (HP) */
static uint8_t  tx_counter_mp  = 0U; /* Compteur cyclique canal 0x101 (MP) */
static uint8_t  tx_counter_lp  = 0U; /* Compteur cyclique canal 0x102 (LP) */
uint8_t         active_counter = 0U;
static CANBOX_Process_e        canbox_process        = PROCESS_BOX_LP;
static IDENTIFICATION_Status_e identification_status = NOT_IDENTIFIED;
static uint32_t                raw_data_32b          = 0UL;
static uint8_t                 Device_short_id       = 0U;
static uint8_t                 status_to_send        = 0U;
static uint32_t                led_counter           = 0UL;
static bool                    flagCAN               = false;
static MOTOR_Status_e          prev_motor_state      = MOTOR_STOP_IDLE;
/* END VARIABLES */

/* CONFIGURATION LED */
if ((led_counter >= 500UL) && flagCAN) {
  led_counter = 0UL;
  flagCAN     = false;
  GPIO_togglePin(20U);
}
led_counter++;
/* END CONFIGURATION LED */

/* ========================================================================== */
/* TX CAN                                                                     */
/* ========================================================================== */

/* PROCESSUS ITÉRATIF DÉPENDANT DU TIMER (Cadencement TX à 10 ms / 100 Hz) */
tx_timer_cnt++;

if (tx_timer_cnt >= 10U) {
  tx_timer_cnt = 0U;

  /* DÉCLARATION VARIABLES */
  uint16_t       txMsgData_Unified[4] = {0U, 0U, 0U, 0U};
  uint8_t        crc_input_buffer[6]  = {0U, 0U, 0U, 0U, 0U, 0U};
  uint32_t       current_id           = 0x102UL;
  uint32_t       target_mailbox       = 2UL;
  MOTOR_Status_e current_motor_state  = motorVars_M1.motorState;
  /* END DÉCLARATION VARIABLES */

  /* PROCESSUS IDENTIFICATION DEVICE */
  if (identification_status == IDENTIFIED) {
    ;
  } else if (identification_status == NOT_IDENTIFIED) {
    /* CAN BOX LP */
    canbox_process = PROCESS_BOX_LP;
    raw_data_32b   = FULL_ID;
    status_to_send = (uint8_t)STATE_REQ_IDENTITY;
  } else if (identification_status == IDENTIFICATION_FE) {
    /* CAN BOX LP */
    canbox_process = PROCESS_BOX_LP;
    raw_data_32b   = FULL_ID;
    status_to_send = (uint8_t)STATE_ID_RECEIVED;
  } else if (identification_status == IDENTIFICATION_FD) {
    identification_status = IDENTIFIED;
    status_to_send        = (uint8_t)STATE_IDLE;
    canbox_process        = PROCESS_BOX_MP;
  } else {
    /* État d'identification invalide : bascule sur boîte HP (0x100) */
    canbox_process = PROCESS_BOX_HP;
    status_to_send = (uint8_t)STATUS_ERR_BAD_STATE;
    raw_data_32b   = (uint32_t)identification_status;
  }
  /* END PROCESSUS IDENTIFICATION DEVICE */

  /* PROCESSUS SURVEILLANCE DÉFAUT MOTEUR */
  if ((identification_status == IDENTIFIED) &&
      (current_motor_state == MOTOR_FAULT_STOP) &&
      (prev_motor_state != MOTOR_FAULT_STOP)) {
    /* Bascule immédiate sur boîte HP (0x100) lors du passage en
     * MOTOR_FAULT_STOP */
    flagCAN        = true;
    canbox_process = PROCESS_BOX_HP;
    status_to_send = (uint8_t)STATUS_ERR_MOTOR_FAULT;
    raw_data_32b =
        (uint32_t)
            motorVars_M1.faultMtrNow.all; /* Masque matériel du défaut TI */
  }

  prev_motor_state = current_motor_state;
  /* END PROCESSUS SURVEILLANCE DÉFAUT MOTEUR */

  /* MACHINE À ÉTATS CONFIGURATION CAN BOX */
  switch (canbox_process) {
    case PROCESS_BOX_HP:
      current_id     = (uint32_t)PROCESS_BOX_HP;
      target_mailbox = 5UL;
      canbox_process = PROCESS_BOX_MP;
      active_counter = tx_counter_hp;
      if (tx_counter_hp >= 255U) {
        tx_counter_hp = 0U;
      } else {
        tx_counter_hp++;
      }
      break;

    case PROCESS_BOX_MP:
      current_id     = (uint32_t)PROCESS_BOX_MP;
      target_mailbox = 1UL;
      active_counter = tx_counter_mp;
      if (tx_counter_mp >= 255U) {
        tx_counter_mp = 0U;
      } else {
        tx_counter_mp++;
      }
      break;

    case PROCESS_BOX_LP:
      current_id     = (uint32_t)PROCESS_BOX_LP;
      target_mailbox = 2UL;
      active_counter = tx_counter_lp;
      if (tx_counter_lp >= 255U) {
        tx_counter_lp = 0U;
      } else {
        tx_counter_lp++;
      }
      break;

    default:
      canbox_process = PROCESS_BOX_HP;
      break;
  }
  /* END MACHINE À ÉTATS CONFIGURATION CAN BOX */

  /* MODIFICATION PAYLOAD DATA 3 BYTES ET 1 BYTE: SHORT_ID */
  raw_data_32b = (uint32_t)(((uint32_t)Device_short_id) << 24U) |
                 (uint32_t)(raw_data_32b & 0x00FFFFFFU);
  /* END MODIFICATION PAYLOAD DATA 3 BYTES ET 1 BYTE: SHORT_ID */

  /* PRÉPARATION DU BUFFER POUR LE CALCUL DU SAFETY CRC16 (INIT = 0xFFFFU) */
  crc_input_buffer[0] = active_counter;
  crc_input_buffer[1] = (uint8_t)(raw_data_32b & 0xFFUL);
  crc_input_buffer[2] = (uint8_t)((raw_data_32b >> 8U) & 0xFFUL);
  crc_input_buffer[3] = (uint8_t)((raw_data_32b >> 16U) & 0xFFUL);
  crc_input_buffer[4] = (uint8_t)((raw_data_32b >> 24U) & 0xFFUL);
  crc_input_buffer[5] = (uint8_t)status_to_send;

  uint16_t safety_crc = Compute_Safety_CRC16(current_id, crc_input_buffer, 6U);
  /* END PRÉPARATION DU BUFFER POUR LE CALCUL DU SAFETY CRC16 (INIT = 0xFFFFU)
   */

  /* TASSEMENT DANS LES MOTS NATIFS DE 16 BITS */
  txMsgData_Unified[0] =
      (uint16_t)active_counter | (uint16_t)((raw_data_32b & 0x000000FFU) << 8U);
  txMsgData_Unified[1] =
      (uint16_t)((raw_data_32b >> 8U) & 0x000000FFU) |
      (uint16_t)(((raw_data_32b >> 16U) & 0x000000FFU) << 8U);
  txMsgData_Unified[2] = (uint16_t)((raw_data_32b >> 24U) & 0x000000FFU) |
                         (uint16_t)(((safety_crc >> 8U) & 0x00FFU) << 8U);
  txMsgData_Unified[3] = (uint16_t)(safety_crc & 0x00FFU) |
                         (uint16_t)((uint16_t)status_to_send << 8U);
  /* END TASSEMENT DANS LES MOTS NATIFS DE 16 BITS */

  /* SEND TX CAN DATA */
  if ((target_mailbox >= 1U) && (target_mailbox <= 32U)) {
    uint32_t mailbox_mask = 1UL << ((uint32_t)target_mailbox - 1UL);

    if ((CAN_getTxRequests(CANA_BASE) & mailbox_mask) == 0UL) {
      CAN_sendMessage_16bit(CANA_BASE, target_mailbox, 8U, txMsgData_Unified);
      CAN_clearInterruptStatus(CANA_BASE, target_mailbox);
    }
  }
  /* END SEND TX CAN DATA */
}
/* END PROCESSUS ITÉRATIF DÉPENDANT DU TIMER */

/* ========================================================================== */
/* RX CAN                                                                     */
/* ========================================================================== */

/* VARIABLES */
static uint16_t rxMsgData[8]     = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
uint8_t         rx_crc_buffer[6] = {0U, 0U, 0U, 0U, 0U, 0U};
uint16_t        received_crc     = 0U;
uint16_t        calculated_crc   = 0U;
uint16_t        crc_msb          = 0U;
uint16_t        crc_lsb          = 0U;
/* END VARIABLES */

/* PROCESSUS RX CAN */
if ((CAN_getNewDataFlags(CANA_BASE) & (1UL << 2U)) != 0UL) {
  /* READ (Mailbox 3 : 1 octet utile par case) */
  CAN_readMessage(CANA_BASE, 3U, rxMsgData);

  /* EXTRACTION DIRECTE DES 6 OCTETS UTILES POUR LE SAFETY CRC16 */
  rx_crc_buffer[0] = (uint8_t)(rxMsgData[0] & 0x00FFU); /* Byte 0 : Counter */
  rx_crc_buffer[1] = (uint8_t)(rxMsgData[1] & 0x00FFU); /* Byte 1 : Spd LSB */
  rx_crc_buffer[2] = (uint8_t)(rxMsgData[2] & 0x00FFU); /* Byte 2 : Spd */
  rx_crc_buffer[3] = (uint8_t)(rxMsgData[3] & 0x00FFU); /* Byte 3 : Spd MSB */
  rx_crc_buffer[4] = (uint8_t)(rxMsgData[4] & 0x00FFU); /* Byte 4 : Short_ID */
  rx_crc_buffer[5] = (uint8_t)(rxMsgData[7] & 0x00FFU); /* Byte 7 : Status */

  /* RECONSTRUCTION DU CRC16 REÇU (BIG ENDIAN : BYTE 5 = MSB, BYTE 6 = LSB) */
  crc_msb      = (uint16_t)((rxMsgData[5] & 0x00FFU) << 8U);
  crc_lsb      = (uint16_t)(rxMsgData[6] & 0x00FFU);
  received_crc = (uint16_t)(crc_msb | crc_lsb);

  /* CALCUL LOCAL DU CRC16 LIÉ À L'ID 0x555 */
  calculated_crc = Compute_Safety_CRC16(0x555U, rx_crc_buffer, 6U);

  /* PROCESSUS SI CRC OK */
  if (calculated_crc == received_crc) {
    uint32_t payload_rx_can_server = 0UL;
    int32_t  payload_data          = 0L;
    flagCAN                        = true;

    /* RECONSTRUCTION UINT32 (Little Endian) */
    payload_rx_can_server = ((uint32_t)rx_crc_buffer[1]) |
                            (((uint32_t)rx_crc_buffer[2]) << 8U) |
                            (((uint32_t)rx_crc_buffer[3]) << 16U) |
                            (((uint32_t)rx_crc_buffer[4]) << 24U);
    /* END RECONSTRUCTION UINT32 (Little Endian) */

    /* PAYLOAD ID (ONLY 3 BYTES) */
    uint32_t payload_id = payload_rx_can_server & 0x00FFFFFFUL;
    /* END PAYLOAD ID (ONLY 3 BYTES) */

    /* EXTENSION DE SIGNE 24 BITS -> 32 BITS SIGNÉS */
    if ((payload_id & 0x00800000UL) != 0UL) {
      payload_data = (int32_t)(payload_id | 0xFF000000UL);
    } else {
      payload_data = (int32_t)payload_id;
    }
    /* END EXTENSION DE SIGNE 24 BITS -> 32 BITS SIGNÉS */

    /* PROCESSUS IDENTIFICATION DEVICE */
    if ((payload_id == FULL_ID) && (identification_status != IDENTIFIED)) {
      if ((rx_crc_buffer[5] == (uint8_t)IDENTIFICATION_FE) &&
          (identification_status == NOT_IDENTIFIED)) {
        Device_short_id       = rx_crc_buffer[4];
        identification_status = IDENTIFICATION_FE;
      }
      if ((rx_crc_buffer[5] == (uint8_t)IDENTIFICATION_FD) &&
          (identification_status == IDENTIFICATION_FE) &&
          (Device_short_id == rx_crc_buffer[4])) {
        identification_status = IDENTIFICATION_FD;
      }
    }
    /* END PROCESSUS IDENTIFICATION DEVICE */

    /* NORMAL PROCESS */
    if ((rx_crc_buffer[4] == Device_short_id) &&
        (identification_status == IDENTIFIED)) {
      uint8_t   status_cmd       = rx_crc_buffer[5];
      float32_t speed_limitation = 70.0f;

      switch (status_cmd) {
        case START_MOTOR:
          motorVars_M1.flagEnableRunAndIdentify = true;
          break;

        case GET_START_MOTOR:
          status_to_send = (uint8_t)GET_START_MOTOR;
          raw_data_32b   = (uint32_t)motorVars_M1.flagEnableRunAndIdentify;
          break;

        case STOP_MOTOR:
          motorVars_M1.flagEnableRunAndIdentify = false;
          break;

        case GET_STOP_MOTOR:
          status_to_send = (uint8_t)GET_STOP_MOTOR;
          raw_data_32b   = (uint32_t)motorVars_M1.flagEnableRunAndIdentify;
          break;

        case SET_SPEED: {
          float32_t vitesse_reelle = (float32_t)payload_data / 1000.0f;

          if ((vitesse_reelle >= -speed_limitation) &&
              (vitesse_reelle <= speed_limitation)) {
            motorVars_M1.speedRef_Hz = vitesse_reelle;
          } else {
            /* Consigne hors limites : maintien consigne + alerte HP (0x100) */
            status_to_send = (uint8_t)SET_SPEED;
            raw_data_32b   = (uint32_t)payload_data;
            canbox_process = PROCESS_BOX_HP;
          }
          break;
        }

        case GET_SPEED:
          status_to_send = (uint8_t)GET_SPEED;
          raw_data_32b   = Pack_Float_To_Int24_Safe(motorVars_M1.speed_Hz);
          break;

        case STATE_SERVER_IDLE:
          /* Trame cyclique de repos du CAN Server (0xF0) : ignorée sans alerte
           */
          break;

        default:
          /* Commande inconnue : maintien consigne moteur + alerte protocole sur
           * HP (0x100) */
          status_to_send = (uint8_t)STATUS_ERR_UNKNOWN_CMD;
          raw_data_32b   = (uint32_t)status_cmd;
          canbox_process = PROCESS_BOX_HP;
          break;
      }
    }
    /* END NORMAL PROCESS */
  }
  /* END PROCESSUS SI CRC OK */

  /* PROCESSUS SI CRC ERREUR */
  else {
    /* Rejet de la trame sans modifier le moteur + alerte CRC sur HP (0x100) si
     * noeud identifié */
    if (identification_status == IDENTIFIED) {
      flagCAN        = true;
      status_to_send = (uint8_t)STATUS_ERR_CRC;
      raw_data_32b   = (uint32_t)received_crc;
      canbox_process = PROCESS_BOX_HP;
    }
  }
  /* END PROCESSUS SI CRC ERREUR */

  /* ACQUITTEMENT MATÉRIEL OBLIGATOIRE */
  CAN_clearInterruptStatus(CANA_BASE, 3U);
}
/* END PROCESSUS RX CAN */
/* ========================================================================== */
/* END EXTENSION PROJET CAN                                                   */
/* ========================================================================== */
