/**
 * @file    can_main_snippet.h
 * @brief   Définitions protocolaires et utilitaires de sûreté CAN à coller dans
 * sys_main.h.
 * @details Destiné au projet TI C2000Ware MotorControl SDK — Universal Motor
 * Control Lab (Build Level 3) sur carte LAUNCHXL-F2800137 + BOOSTXL-DRV8323RS
 * (Site 1 : J1/J3/J4/J2).
 *
 * @copyright Copyright (c) 2026 Jorge Aguilar - Extensions CAN Protocol.
 *            Distribué sous licence Apache License 2.0 (voir fichier LICENSE).
 */

/* ========================================================================== */
/* À COLLER EN FIN DE sys_main.h (avant le #endif final)                      */
/* ========================================================================== */

/* ========================================================================== */
/* EXTENSION PROJET CAN — Copyright (c) 2026 Jorge Aguilar (Licence Apache 2.0)
 */
/* ========================================================================== */

/**
 * @def   FULL_ID
 * @brief Identifiant matériel unique sur 24 bits du Target Node TI C2000
 * (utilisé lors de l'énumération dynamique).
 */
#define FULL_ID 0xC2000UL

/**
 * @struct  CAN_TX_data_t
 * @brief   Structure applicative de contexte et de sérialisation d'une trame
 * CAN.
 * @details Regroupe les champs utiles de la trame 8 octets (hors CRC-16 calculé
 * dynamiquement) ainsi que l'identifiant matériel complet du Target Node.
 */
typedef struct {
  uint8_t counter; /**< Compteur cyclique de trames (0 à 255) pour la détection
                      de perte de paquets. */
  uint8_t short_id; /**< Adresse logique courte attribuée au Target Node (0x01 à
                       0xFE, ou 0xFF en broadcast). */
  uint8_t data0;    /**< Octet de poids faible (LSB) de la charge utile signée
                       int24 (Little Endian). */
  uint8_t data1;    /**< Octet intermédiaire de la charge utile signée int24
                       (Little Endian). */
  uint8_t data2;    /**< Octet de poids fort (MSB, bit de signe en bit 23) de la
                       charge utile int24. */
  uint8_t status; /**< Code commande reçu ou statut / acquittement courant émis
                     (Octet 7). */
  int32_t full_id; /**< Identifiant matériel unique sur 24 bits du Target Node
                      (ex. FULL_ID = 0x0C2000). */

} CAN_TX_data_t;

/**
 * @enum    SetInstructions_e
 * @brief   Codes de commandes et de télémétrie moteur (plage 0x10 - 0x1F de
 * l'octet Status).
 * @details Définit les instructions transmises par le CAN Server (piloté par le
 * Test Host HIL) pour commander la boucle InstaSPIN-FOC ou interroger l'état
 * temps réel du moteur brushless.
 */
typedef enum {
  START_MOTOR = 0x10U, /**< Commande d'activation du contrôle moteur
                          (flagEnableRunAndIdentify = true). */
  GET_START_MOTOR =
      0x11U, /**< Lecture / acquittement de l'état d'activation du moteur. */
  STOP_MOTOR = 0x12U, /**< Commande d'arrêt du contrôle moteur
                         (flagEnableRunAndIdentify = false). */
  GET_STOP_MOTOR =
      0x13U,         /**< Lecture / acquittement de l'état d'arrêt du moteur. */
  SET_SPEED = 0x14U, /**< Écriture de la consigne de vitesse en Hz (encodée en
                        int24 x1000, bornée à ±70,0 Hz). */
  GET_SPEED = 0x15U /**< Requête / remontée de la vitesse électrique mesurée
                       (speed_Hz en int24 x1000). */

} SetInstructions_e;

/**
 * @enum    IDENTIFICATION_Status_e
 * @brief   États internes de la machine à états d'identification du Target
 * Node.
 * @details Gère la progression de l'enregistrement dynamique du Target Node sur
 * le bus CAN jusqu'à l'attribution définitive de son adresse courte (@c
 * short_id) par le CAN Server.
 */
typedef enum {
  NOT_IDENTIFIED = 0x00U, /**< Target Node non encore enregistré sur le bus (en
                             attente d'attribution). */
  IDENTIFIED = 0x01U, /**< Target Node enregistré et opérationnel (filtre actif
                         sur son short_id). */
  IDENTIFICATION_FE = 0xFEU, /**< Étape intermédiaire : adresse courte reçue, en
                                attente de validation du CAN Server. */
  IDENTIFICATION_FD = 0xFDU /**< Étape finale : validation du CAN Server
                               confirmée, prêt à basculer en mode nominal. */

} IDENTIFICATION_Status_e;

/**
 * @enum    CANBOX_Process_e
 * @brief   Identifiants CAN 11 bits associés aux canaux de priorité d'émission
 * (Target Node -> CAN Server).
 * @details Exploite l'arbitrage matériel non destructif du bus CAN 2.0A en
 * séparant les boîtes aux lettres DCAN selon la criticité temporelle des
 * messages.
 */
typedef enum {
  PROCESS_BOX_HP =
      0x100U, /**< Canal Haute Priorité (Mailbox 5) : alertes critiques, erreurs
                 protocole et défauts matériels. */
  PROCESS_BOX_MP = 0x101U, /**< Canal Moyenne Priorité (Mailbox 1) : télémétrie
                              temps réel (vitesse moteur mesurée). */
  PROCESS_BOX_LP = 0x102U /**< Canal Basse Priorité (Mailbox 2) : énumération
                             dynamique, statuts et diagnostic lent. */

} CANBOX_Process_e;

/**
 * @enum    STATUS_Value_e
 * @brief   Codes protocole du domaine d'énumération réseau (plage 0xF0 - 0xFF
 * de l'octet Status).
 * @details Utilisés lors de la séquence de découverte et d'attribution
 * dynamique d'adresse au démarrage.
 */
typedef enum {
  /* Domaine Identification & États Protocole CAN (0xF0 - 0xFF) */
  STATE_SERVER_IDLE = 0xF0U, /**< État de repos nominal CAN Server (ignoré
                                silencieusement par le Target Node). */
  STATE_IDLE =
      0xFCU, /**< État de repos nominal côté Target Node (distinct du 0xF0 du
                CAN Server pour éviter tout déclenchement UART parasite). */
  STATE_ACK = 0xFDU, /**< Validation finale de l'identification émise par le CAN
                        Server. */
  STATE_ID_RECEIVED =
      0xFEU, /**< Attribution / acquittement de l'adresse courte (short_id). */
  STATE_REQ_IDENTITY = 0xFFU /**< Requête initiale d'identification émise par le
                                Target Node au démarrage. */

} STATUS_Value_e;

/**
 * @enum    ProtocolErrors_e
 * @brief   Codes d'erreurs protocole et transport (plage transverse 0xE0 - 0xEF
 * de l'octet Status).
 * @details Remontés en priorité sur le canal Haute Priorité (@c PROCESS_BOX_HP
 * = 0x100) vers le CAN Server lorsqu'une trame est rejetée avant ou pendant son
 * décodage applicatif.
 */
typedef enum {
  STATUS_ERR_GENERIC = 0xE0U, /**< Erreur protocole générique non spécifiée. */
  STATUS_ERR_CRC =
      0xE1U, /**< Erreur d'intégrité Safety_CRC16 sur trame reçue. */
  STATUS_ERR_OUT_RANGE =
      0xE2U, /**< Paramètre reçu hors plage physique autorisée. */
  STATUS_ERR_BAD_STATE = 0xE3U,   /**< Commande reçue incompatible avec l'état
                                     courant du Target Node. */
  STATUS_ERR_MOTOR_FAULT = 0xE4U, /**< Arrêt moteur sur défaut InstaSPIN
                                     (motorState == MOTOR_FAULT_STOP). */
  STATUS_ERR_UNKNOWN_CMD =
      0xEEU /**< Code commande inconnu reçu dans le bloc applicatif. */

} ProtocolErrors_e;

/**
 * @brief   Calcule un CRC-16/CCITT-FALSE sécuritaire sur l'identifiant et les
 * données d'une trame CAN.
 *
 * @details Implémente le calcul d'intégrité selon le polynôme CRC-16/CCITT
 * (0x1021) avec une valeur initiale de 0xFFFF (sans réflexion ni XOR final).
 * L'empreinte inclut les 2 octets de poids faible de l'identifiant CAN suivis
 * des octets utiles de la trame (jusqu'à 8 octets). En cas de paramètre
 * invalide (pointeur nul ou longueur > 8), applique un repli sécuritaire
 *          conforme aux exigences EN 50128 en retournant la valeur initiale
 * 0xFFFF.
 *
 * @param[in] can_id  Identifiant de la trame CAN (seuls les 16 bits de poids
 * faible sont intégrés au calcul).
 * @param[in] data    Pointeur constant vers le tableau d'octets de charge utile
 * (payload CAN).
 * @param[in] length  Nombre d'octets valides dans @p data (plage valide : 0 à 8
 * inclus).
 *
 * @return  uint16_t  Valeur du CRC-16 calculé, ou @c 0xFFFFU en cas de repli
 * sécuritaire (entrée invalide).
 *
 * @pre     @p data doit pointer vers une zone mémoire valide d'au moins @p
 * length octets.
 * @note    Fonction déterministe sans allocation dynamique ni effet de bord
 * (compatible MISRA-C / EN 50128).
 */
static inline uint16_t Compute_Safety_CRC16(uint32_t             can_id,
                                            const uint8_t* const data,
                                            uint16_t             length) {
  uint16_t crc = 0xFFFFU;

  if ((data != NULL) && (length <= 8U)) {
    uint16_t i           = 0U;
    uint16_t bit         = 0U;
    uint8_t  id_bytes[2] = {0U, 0U};

    id_bytes[0] = (uint8_t)((can_id >> 8U) & 0xFFUL);
    id_bytes[1] = (uint8_t)(can_id & 0xFFUL);

    for (i = 0U; i < 2U; i++) {
      crc ^= (uint16_t)(((uint16_t)id_bytes[i]) << 8U);

      for (bit = 0U; bit < 8U; bit++) {
        if ((crc & 0x8000U) != 0U) {
          crc = (uint16_t)((crc << 1U) ^ 0x1021U);
        } else {
          crc = (uint16_t)(crc << 1U);
        }
      }
    }

    for (i = 0U; i < length; i++) {
      crc ^= (uint16_t)(((uint16_t)data[i]) << 8U);

      for (bit = 0U; bit < 8U; bit++) {
        if ((crc & 0x8000U) != 0U) {
          crc = (uint16_t)((crc << 1U) ^ 0x1021U);
        } else {
          crc = (uint16_t)(crc << 1U);
        }
      }
    }
  } else {
    /* Repli sécuritaire EN 50128 en cas de pointeur nul ou longueur invalide */
    crc = 0xFFFFU;
  }

  return crc;
}

/**
 * @brief  Convertit de manière sécurisée un flottant physique (x1000) vers un
 *         mot 32 bits encapsulant un entier signé 24 bits (int24).
 * @note   Rejette automatiquement NaN, +/-Inf et tout débordement hors
 *         [-8388.608f ; +8388.607f] (repli sécurisé à 0UL).
 * @param  value_phys Grandeur physique en float32_t (ex. vitesse en Hz).
 * @return uint32_t   Représentation binaire 32 bits (int32_t casté) prête
 *                    pour l'extraction des 3 octets int24.
 */
static inline uint32_t Pack_Float_To_Int24_Safe(float32_t value_phys) {
  uint32_t packed_data = 0UL;

  /* En IEEE-754, toute comparaison avec NaN ou +/-Inf hors bornes retourne
   * false */
  if ((value_phys >= -8388.608f) && (value_phys <= 8388.607f)) {
    int32_t scaled_val = (int32_t)(value_phys * 1000.0f);
    packed_data        = (uint32_t)scaled_val;
  } else {
    /* Repli sécuritaire EN 50128 en cas de valeur invalide ou hors plage */
    packed_data = 0UL;
  }

  return packed_data;
}

/* ========================================================================== */
/* END EXTENSION PROJET CAN                                                   */
/* ========================================================================== */
