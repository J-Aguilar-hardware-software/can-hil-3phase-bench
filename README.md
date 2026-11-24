# Banc de Validation HIL & Bus CAN Temps Réel (TI C2000 · STM32 · Python)

[![Phase 1](https://shields.io)](src/)
[![Phase 2](https://shields.io)](#7-encapsulation-passerelle-uart-test-host--can-server)
[![Phase 3](https://shields.io)](#9-tester-la-phase-1-can-driver-c2000--feuille-de-route)
[![Standard](https://shields.io)](misra_compliance.md)
[![License](https://shields.io)](LICENSE)


**Architecture distribuée de pilotage et de test automatisé Hardware-in-the-Loop (`Test Host` ↔ `CAN Server` ↔ `Target Node`). Ce dépôt présente actuellement la Phase 1 : le driver applicatif et protocole CAN 2.0A déterministe développé pour microcontrôleur TI C2000 (`TMS320F2800137`) sous contraintes MISRA C:2012 / EN 50128.**

[**Explorer le code source Phase 1 (`src/`) →**](src/) · [**Synthèse Conformité MISRA C / EN 50128 →**](misra_compliance.md)

---

### Avancement du Projet par Phases

| Phase | Composant | Rôle dans l'Architecture | Statut |
| :--- | :--- | :--- | :--- |
| **Phase 1** | **CAN Driver & Protocole (`Target Node` TI C2000)** | Driver applicatif CAN 2.0A déterministe (`1 ms`), arbitrage matériel (`HP`/`MP`/`LP`), encodage `int24`, `Safety_CRC16` et interface moteur **InstaSPIN-FOC**. | **Disponible (`v1.0`)** |
| **Phase 2** | **`CAN Server` (`NUCLEO-F446RE` STM32)** | Passerelle temps réel UART ↔ CAN, gestion dynamique des périphériques (`Short_ID`), routage des priorités et supervision bus. | *Prochainement (`v1.1`)* |
| **Phase 3** | **`Test Host` (Banc Python / `pytest`)** | Séquenceur de tests automatisés HIL, validation en boucle fermée, injection de défauts et génération de rapports. | *Prochainement (`v1.2`)* |

---

## Sommaire
1. [Architecture & Configuration Matérielle (HIL)](#1-architecture--configuration-matérielle-hil)
2. [Vue d'ensemble & Architecture Sûreté (Phase 1 : CAN Driver)](#2-vue-densemble--architecture-sûreté-phase-1--can-driver)
3. [Spécification de la Couche Physique & Arbitrage CAN](#3-spécification-de-la-couche-physique--arbitrage-can)
4. [Structure de la Trame CAN (8 Octets) & Encodage `int24`](#4-structure-de-la-trame-can-8-octets--encodage-int24)
5. [Intégrité & Sécurité (`Safety_CRC16`)](#5-intégrité--sécurité-safety_crc16)
6. [Plan d'Adressage du Champ `Status` & Énumération Dynamique](#6-plan-dadressage-du-champ-status--énumération-dynamique)
7. [Encapsulation Passerelle UART (`Test Host` ↔ `CAN Server`)](#7-encapsulation-passerelle-uart-test-host--can-server)
8. [Traces d'Exécution Réelles sur Matériel (Initialisation & Résilience *Hot-Reset*)](#8-traces-dexécution-réelles-sur-matériel-initialisation--résilience-hot-reset)
9. [Tester la Phase 1 (`CAN Driver` C2000) & Feuille de Route](#9-tester-la-phase-1-can-driver-c2000--feuille-de-route)
10. [Licence & Propriété Intellectuelle](#10-licence--propriété-intellectuelle)

---

## 1. Architecture & Configuration Matérielle (HIL)

### Topologie du Banc d'Essai
```text
[Test Host (PC / pytest)] <--- UART 115200 bps (12 octets) ---> [CAN Server (NUCLEO-F446RE)] <--- Bus CAN 500 kbps (8 octets) ---> [Target Node (TI C2000 + BLDC)]
```

### Matériel & Environnement Logiciel
* **`Target Node` (Phase 1 — Contrôle Moteur) :** Carte d'évaluation **Texas Instruments `LAUNCHXL-F2800137`** avec l'étage de puissance triphasé **`BOOSTXL-DRV8323RS`** monté sur le **connecteur BoosterPack Site 1 (`J1 / J3 / J4 / J2`)**.
* **Base Logicielle FOC :** **TI C2000Ware MotorControl SDK — Universal Motor Control Lab (`Build Level 3`)**, exécutant la régulation de vitesse en boucle fermée sans capteur via l'estimateur **FAST / InstaSPIN-FOC**.
* **`CAN Server` (Phase 2 — Passerelle & Coordinateur Réseau) :** Carte **STMicroelectronics `NUCLEO-F446RE` (`STM32F446RE`)** assurant la conversion bidirectionnelle UART (`115200 bauds`) ↔ CAN (`500 kbps`), la gestion de la table des périphériques et l'attribution dynamique des adresses courtes (`Short_ID`).
* **Interface Physique CAN :** Transceiver CAN **3,3 V** (ex. SN65HVD230 / TCAN332) raccordé sur `GPIO12` (`CANA_RX`) et `GPIO13` (`CANA_TX`) avec masse commune (`GND`) et terminaisons `120 Ω`.
* **`Test Host` (Phase 3 — Superviseur HIL) :** Exécution automatisée des scénarios de validation séquentiels sous Python (`pytest`).

### Routage & Multiplexage des Broches CAN (`TMS320F2800137`)
Sur le `TMS320F2800137`, les broches **`GPIO12`** et **`GPIO13`** sont des broches mixtes analogiques/numériques (**AGPIO**). Le périphérique `DCANA` est routé via la position de multiplexage `0xD` (`pin_map.h`) après désactivation explicite du chemin analogique (`GPIO_ANALOG_DISABLED`) :

```c
/* Extrait driverlib/f280013x/driverlib/pin_map.h */
#define GPIO_12_CANA_RX    0x0006180DU   /* Entrée réception CAN (GPIO_QUAL_ASYNC + Pull-Up) */
#define GPIO_13_CANA_TX    0x00061A0DU   /* Sortie émission CAN vers transceiver 3,3 V */
```

---

## 2. Vue d'ensemble & Architecture Sûreté (Phase 1 : CAN Driver)

La Phase 1 implémente un driver applicatif et une pile protocolaire déterministe au sein d'une boucle de contrôle moteur temps réel sur **TI C2000 (`TMS320F2800137`)** :

| Domaine | Choix Technique | Bénéfice Système |
| :--- | :--- | :--- |
| **Déterminisme Temps Réel** | Scrutation RX à `1 ms` (`1 kHz`) et émission TX cadencée à `10 ms` (`100 Hz`) dans `HAL_CPU_TIMER0` | Élimination des conditions de course (*race conditions*), réactivité immédiate aux commandes/alarmes et maîtrise de la charge bus (`2,6 %` CAN / `10,4 %` UART). |
| **Qualité & Normes** | Développement guidé par **MISRA C:2012** et **EN 50128** sur cœur C28x | Typage strict (`U`/`UL`), portée minimale (`static`), zéro code mort, protection IEEE-754 (`Pack_Float_To_Int24_Safe`) et saturation physique (`±70,0 Hz`). |
| **Protocole CAN Compact** | Trame 8 octets avec donnée signée **`int24`** et **`Safety_CRC16`** | Transport de grandeurs physiques haute résolution (`× 1000`) avec vérification d'intégrité incluant l'ID CAN. |
| **Arbitrage Matériel** | Séparation des boîtes aux lettres : `HP` (`0x100`), `MP` (`0x101`), `LP` (`0x102`) | Priorité garantie aux alarmes critiques (`MOTOR_FAULT_STOP`, erreurs protocole) sur la télémétrie et le diagnostic lent. |
| **Énumération Dynamique** | Machine à états `0xFF` → `0xFE` → `0xFD` avec attribution de `Short_ID` | Découverte automatique des `Target Nodes` et réattribution déterministe en cas de *reset* à chaud. |

> **Dimensionnement Temporel (`1 ms` RX / `10 ms` TX) :**
> La réception est scrutée à chaque tick de `1 ms` (`1 kHz`) afin d'exécuter les consignes moteur sans latence. L'émission TX est cadencée à **`10 ms` (`100 Hz`)** : ce compromis assure une excellente résolution dynamique tout en maintenant la charge du bus CAN (`500 kbps`) à **`2,6 %`** et celle du lien passerelle UART (`115200 bauds`) à **`10,4 %`**, évitant toute saturation lors des variations continues de vitesse.

---


## 3. Spécification de la Couche Physique & Arbitrage CAN

### Paramètres Bus
* **Standard :** CAN 2.0A (Identifiants Standard **11 bits**)
* **Débit binaire (*Bitrate*) :** `500 kbps` (`20 TQ` à `SYSCLK = 120 MHz`)
* **Point d'échantillonnage (*Sample Point*) :** `75 %` à `87,5 %`
* **Longueur utile (*DLC*) :** Fixe à **8 octets** par trame
* **Terminaison de ligne :** Deux résistances de `120 Ω` aux extrémités physiques de la paire différentielle `CAN_H` / `CAN_L`

### Arbitrage Matériel par Priorité
L'architecture exploite l'arbitrage non destructif natif du bus CAN pour garantir la priorité des flux critiques sur le diagnostic réseau :

| Identifiant CAN | Direction | Boîte aux Lettres (C2000) | Canal / Priorité | Rôle Fonctionnel |
| :--- | :--- | :--- | :--- | :--- |
| **`0x100`** | `Target Node` → `CAN Server` | `CAN_BOX_HP` (Mailbox 5) | **Haute Priorité (HP)** | Alarmes moteur (`MOTOR_FAULT_STOP`), échecs d'exécution et erreurs protocole (`0xE0`–`0xEF`). |
| **`0x101`** | `Target Node` → `CAN Server` | `CAN_BOX_MP` (Mailbox 1) | **Moyenne Priorité (MP)** | Télémétrie temps réel (ex. vitesse moteur mesurée `speed_Hz`). |
| **`0x102`** | `Target Node` → `CAN Server` | `CAN_BOX_LP` (Mailbox 2) | **Basse Priorité (LP)** | Énumération réseau, statut d'activation moteur et diagnostic lent. |
| **`0x555`** | `CAN Server` → `Target Nodes` | `CAN_BOX_RX` (Mailbox 3) | **Canal de Commande** | Diffusion des requêtes d'identification et des consignes de pilotage (filtre exact `0x7FF`). |

> **Filtrage Matériel Silicium (`0x555` / Masque `0x7FF`) :**
> La boîte de réception `CAN_BOX_RX` (Mailbox 3) est configurée avec un masque d'acceptation strict sur les 11 bits (`0x7FFUL`). Le contrôleur matériel DCAN filtre ainsi automatiquement tout le trafic montant des autres `Target Nodes` (`0x100`–`0x102`) sans consommer de cycles CPU sur la boucle de contrôle moteur.

---

## 4. Structure de la Trame CAN (8 Octets) & Encodage `int24`

Chaque trame échangée sur le bus respecte strictement la disposition suivante :

| Octet | Champ | Type | Endianness | Description |
| :---: | :--- | :--- | :--- | :--- |
| **0** | `n_packet_counter` | `uint8_t` | — | Compteur cyclique de trames (`0` à `255`) permettant de détecter les pertes de paquets. |
| **1** | `payload_b0` (LSB) | `int24` | Little Endian | Octet de poids faible de la donnée utile signée 24 bits. |
| **2** | `payload_b1` | `int24` | Little Endian | Octet intermédiaire de la donnée utile signée 24 bits. |
| **3** | `payload_b2` (MSB) | `int24` | Little Endian | Octet de poids fort (incluant le bit de signe `0x80` en bit 23). |
| **4** | `Short_ID` | `uint8_t` | — | Adresse logique courte attribuée au `Target Node` (`0x01` à `0xFE`), ou `0xFF` en broadcast. |
| **5** | `Safety_CRC16_H` | `uint16_t` | Big Endian | Octet de poids fort de la somme de contrôle de sécurité. |
| **6** | `Safety_CRC16_L` | `uint16_t` | Big Endian | Octet de poids faible de la somme de contrôle de sécurité. |
| **7** | `Status` | `uint8_t` | — | Code commande (`CAN Server` → `Target Node`) ou état courant / acquittement (`Target Node` → `CAN Server`). |

### Encodage de la Charge Utile Signée (`int24`)
* **Plage dynamique :** `[-8 388 608 ; +8 388 607]`
* **Mise à l'échelle physique :** Les grandeurs physiques (vitesse en `Hz`, courants, tensions) sont multipliées par **`1000`** avant encodage, offrant une résolution de `0,001` sur une plage de **`± 8 388,607`**.
* **Extension de signe en réception (24 bits → 32 bits) :** Si le bit 23 (`0x00800000`) est à `1`, l'octet supérieur est complété à `0xFF` (`payload_u32 | 0xFF000000UL`).

---

## 5. Intégrité & Sécurité (`Safety_CRC16`)

Afin de garantir qu'aucune trame corrompue ni aucun message mal routé ne puisse modifier la consigne d'un actionneur, chaque trame intègre une somme de contrôle **CRC-16/CCITT-FALSE** calculée sur **8 octets** (les **2 octets de poids faible de l'identifiant CAN** + les **6 octets utiles de la charge utile**).

### Paramètres Algorithmiques
* **Algorithme :** CRC-16/CCITT-FALSE
* **Polynôme générateur :** `0x1021`
* **Valeur initiale (`Init`) :** `0xFFFF`
* **Réflexion entrée / sortie (`RefIn` / `RefOut`) :** `false` / `false`
* **XOR final (`XorOut`) :** `0x0000`
* **Vecteur d'entrée (`8 octets`) :** `[CAN_ID_MSB, CAN_ID_LSB, Byte0_Counter, Byte1_Data0, Byte2_Data1, Byte3_Data2, Byte4_ShortID, Byte7_Status]`

### Propriétés de Sûreté (EN 50128)
* **Protection contre l'usurpation d'identité (*Masquerading*) :** L'inclusion des octets `CAN_ID_MSB` et `CAN_ID_LSB` dans le calcul empêche qu'une trame destinée à une boîte aux lettres (`0x102 LP`) soit acceptée comme une commande critique (`0x555 RX`) ou une alarme (`0x100 HP`).
* **Repli sécuritaire déterministe :** En cas de pointeur nul (`data == NULL`) ou de longueur invalide (`length > 8U`), la fonction `Compute_Safety_CRC16()` retourne immédiatement la valeur initiale de repli `0xFFFFU`.
* **Rejet silencieux + alerte :** Toute trame reçue avec un CRC invalide est rejetée sans modifier l'état du moteur et déclenche une remontée d'erreur `0xE1` (`STATUS_ERR_CRC`) sur le canal haute priorité `0x100`.

---


## 6. Plan d'Adressage du Champ `Status` & Énumération Dynamique

### Plan d'Adressage Modulaire (Octet 7 — Blocs de 16 Codes)
L'espace d'adressage (256 codes) est structuré par **profils fonctionnels de 16 valeurs (`0x10`)** afin de permettre l'ajout de nouveaux types de `Target Nodes` sur le banc HIL sans jamais saturer le protocole. Par convention, les **codes pairs (`bit 0 = 0`)** désignent une action/consigne (`SET`) et les **codes impairs suivants (`bit 0 = 1`)** désignent une requête/télémétrie (`GET`) :

| Plage Hex | Bloc (`Status >> 4`) | Profil / Domaine Fonctionnel | Codes Définis & Exemples d'Extension |
| :--- | :---: | :--- | :--- |
| **`0x00 – 0x0F`** | `0x0` | **Système & Réseau (`CAN Server` — *Reserved*)** | Plage réservée aux commandes d'administration globale du coordinateur `CAN Server` (`0x00` : `DISCOVERY_BROADCAST` — requête générale de présence sur le bus ; `0x01 – 0x0F` : réservés pour évolutions système). |
| **`0x10` – `0x1F`** | `0x1` | **Contrôle Moteur (BLDC / PMSM)** | `0x10` : `START_MOTOR` \| `0x11` : `GET_START_MOTOR`<br>`0x12` : `STOP_MOTOR` \| `0x13` : `GET_STOP_MOTOR`<br>`0x14` : `SET_SPEED` \| `0x15` : `GET_SPEED` (`±70,0 Hz` max) |
| **`0x20` – `0x2F`** | `0x2` | **Actionneurs & Puissance** *(Réservé)* | Relais, électrovannes, PWM, étages de puissance (`SET_GPIO`, `SET_PWM_DUTY`). |
| **`0x30` – `0x3F`** | `0x3` | **Capteurs Environnementaux** *(Réservé)* | Température, pression, débit (`GET_TEMPERATURE`, `GET_PRESSURE`). |
| **`0x40` – `0x4F`** | `0x4` | **Capteurs Électriques & Énergie** *(Réservé)* | Alimentations, BMS (`GET_VOLTAGE_DC`, `GET_CURRENT_DC`, `GET_SOC`). |
| **`0x50` – `0x5F`** | `0x5` | **Capteurs Mécaniques & Position** *(Réservé)* | Codeurs, IMU (`GET_POSITION`, `GET_ANGLE`, `GET_ACCEL_XYZ`). |
| **`0x60` – `0xDF`** | `0x6–0xD` | **Réserve Projet (128 codes)** | Profils métiers spécifiques et extensions futures. |
| **`0xE0` – `0xEF`** | `0xE` | **Erreurs Protocole & Alarmes Matérielles** | `0xE0` : `STATUS_ERR_GENERIC` \| `0xE1` : `STATUS_ERR_CRC`<br>`0xE2` : `STATUS_ERR_OUT_RANGE` \| `0xE3` : `STATUS_ERR_BAD_STATE`<br>`0xE4` : `STATUS_ERR_MOTOR_FAULT` (`MOTOR_FAULT_STOP`)<br>`0xEE` : `STATUS_ERR_UNKNOWN_CMD` (remontée sur `0x100 HP`) |
| **`0xF0` – `0xFF`** | `0xF` | **Énumération & États Réseau** | `0xF0` : `SERVER_IDLE` \| `0xFC` : `STATE_IDLE` (`Target Node`)<br>`0xFD` : `STATE_ACK` \| `0xFE` : `STATE_ID_RECEIVED` \| `0xFF` : `STATE_REQ_IDENTITY` |

> **Diagnostic Contextuel `(CAN_ID, Status)` :**
> * **Alarme moteur & échec d'exécution :** Lorsqu'un défaut matériel InstaSPIN arrête le moteur (`motorVars_M1.motorState == MOTOR_FAULT_STOP`), le `Target Node` bascule immédiatement sur la boîte haute priorité **`0x100` (`PROCESS_BOX_HP`)** avec le code d'état **`0xE4` (`STATUS_ERR_MOTOR_FAULT`)** (la remontée détaillée du masque de faute matériel dans les 3 octets de payload étant prévue pour l'évolution suivante). De même, si une commande valide échoue (ex. consigne `SET_SPEED` hors limites `±70,0 Hz`), le `Target Node` maintient sa consigne sûre et remonte le tuple `(0x100, SET_SPEED)`.
> * **Erreur protocole :** Toute trame corrompue (`0xE1`) ou commande inconnue (`0xEE`) est rejetée sans interrompre le moteur et signalée sur **`0x100` (`PROCESS_BOX_HP`)**.
> * **Isolation `IDLE` (`0xF0` vs `0xFC`) :** L'état de repos du `Target Node` (`0xFC`) est distinct de celui du `CAN Server` (`0xF0`) afin d'éviter tout déclenchement parasite du flux UART vers le `Test Host`.

### Séquence d'Énumération Dynamique (`0xFF` → `0xFE` → `0xFD`)
À la mise sous tension, aucun `Target Node` ne possède d'adresse courte fixe. Le `CAN Server` découvre chaque nœud grâce à son identifiant matériel unique sur 24 bits (`FULL_ID`, ex. `0x0C2000`). En cas de *reset* à chaud du `Target Node`, le `CAN Server` reconnaît le `FULL_ID` existant et lui réattribue de façon déterministe le même `Short_ID` :

```text
Target Node (TI C2000)                              CAN Server (NUCLEO-F446RE)

       |                                                        |
       |--- [0x102 LP] Payload: FULL_ID | Status: 0xFF -------->|  1. Demande d'identification
       |                                                        |
       |<-- [0x555 RX] Payload: FULL_ID | Short_ID: 0x01 | 0xFE |  2. Attribution du Short_ID
       |                                                        |
       |--- [0x102 LP] Payload: FULL_ID | Status: 0xFE -------->|  3. Acquittement du Short_ID
       |                                                        |
       |<-- [0x555 RX] Payload: FULL_ID | Short_ID: 0x01 | 0xFD |  4. Validation opérationnelle
       |                                                        |
       |--- [0x102 LP] Payload: FULL_ID | Short_ID: 0x01 | 0xFC |  5. Passage en mode IDLE cible (0xFC)
```

---

## 7. Encapsulation Passerelle UART (`Test Host` ↔ `CAN Server`)

Lorsque le `Target Node` C2000 est piloté depuis le `Test Host` via le `CAN Server` STM32 (comme dans le banc HIL), les trames CAN sont encapsulées dans un paquet série de **12 octets** en Little Endian (`<BBHBBHi` à `115200 bauds`) :

| Octets | Champ | Format | Description |
| :---: | :--- | :--- | :--- |
| **0–1** | `Sync Header` | `0xAA 0xBB` | Octets de synchronisation de début de trame. |
| **2–3** | `CAN_ID` | `uint16_t` (LE) | Identifiant de routage ou cible matérielle (ex. `0x0C20`). |
| **4** | `Status` | `uint8_t` | Code commande / registre (`0x10` à `0xFF`). |
| **5** | `Counter` | `uint8_t` | Compteur de trame (`0` à `255`). |
| **6–7** | `UART_CRC16` | `uint16_t` (LE) | Somme de contrôle de la liaison série. |
| **8–11** | `Payload_32` | `int32_t` (LE) | Donnée utile étendue de `int24` vers `int32_t` signé. |

---

## 8. Traces d'Exécution Réelles sur Matériel (Initialisation & Résilience *Hot-Reset*)

Les captures série ci-dessous proviennent directement du coordinateur `CAN Server` (`STM32 NUCLEO-F446RE`) dialoguant à `500 kbps` avec le `Target Node` (`LAUNCHXL-F2800137`). Elles démontrent :
1. **L'enregistrement initial à froid (`0xFF` → `0xFE` → `0xFD`)** avec attribution dynamique de `Short_ID = 0x1` et basculement sur le canal temps réel `0x101` (`MP`).
2. **La résilience lors d'un *reset* à chaud du `Target Node` C2000 :** dès que la cible redémarre et relance sa requête `0xFF` (`=0`) sur `0x102` (`LP`), le `CAN Server` détecte immédiatement que son identifiant matériel est déjà enregistré (`ID: 0xC2000 existe déjà`) et lui **réattribue de manière déterministe la même adresse courte (`Short_ID = 0x1`)** jusqu'à l'acquittement final `0xFD` et la reprise sur `0x101`.

```text
--- 1. ENREGISTREMENT INITIAL À FROID (Target Node C2000 <-> CAN Server STM32) ---
[CAN RX TRAME 0x102]:=1=0x0=0x20=0xC [ID:4]=0x0 [STATUS:7]=0xFF
[LP_PROCESS]: ACTIVATE
[PROCESSUS]: 0xFF
[READ_MAILBOX_LP]: Demande d'identification reçue
PAYLOAD_ID: 0xC2000
NEW ID: 0xC2000
CAN Devices counter_devices = 1
PAYLOAD_COMPLET: 0x10C2000
[CAN_MANAGER_DEVICES]: error_counter: 0

TX_CAN: Data envoyée
[CAN_TX]: Counter: 163 | Short_ID: 0x1 | Status: 0xFE
[CAN RX TRAME 0x102]:=2=0x0=0x20=0xC [ID:4]=0x1 [STATUS:7]=0xFE
[LP_PROCESS]: ACTIVATE
[PROCESSUS]: 0xFE
[CAN_MANAGER_DEVICES]: Device ajouté
[CAN_MANAGER_DEVICES]: full_ID: 0xC2000
[CAN_MANAGER_DEVICES]: Status device: 0xFE
[CAN_MANAGER_DEVICES]: Payload: 0x10C2000
[CAN_MANAGER_DEVICES]: error_counter: 0

TX_CAN: Data envoyée
[CAN_TX]: Counter: 164 | Short_ID: 0x1 | Status: 0xFD
[CAN RX TRAME 0x101]:=4=0x0=0x0=0x0 [ID:4]=0x1 [STATUS:7]=0xFD

--- 2. RESET À CHAUD DU C2000 -> RÉATTRIBUTION DE LA MÊME ADRESSE (ID = 0x1) ---
[CAN RX TRAME 0x102]:=0=0x0=0x20=0xC [ID:4]=0x0 [STATUS:7]=0xFF
[READ_MAILBOX_LP_0xFF]
[PROCESSUS]: 0xFF
[READ_MAILBOX_LP]: Demande d'identification reçue
PAYLOAD_ID: 0xC2000
ID: 0xC2000 existe déjà
PAYLOAD_COMPLET: 0x10C2000

TX_CAN: Data envoyée
[CAN_TX]: Counter: 178 | Short_ID: 0x1 | Status: 0xFE
[CAN RX TRAME 0x102]:=1=0x0=0x20=0xC [ID:4]=0x1 [STATUS:7]=0xFE
[READ_MAILBOX_LP_0xFE]
[PROCESSUS]: 0xFE
[CAN_MANAGER_DEVICES]: Device mis à jour (Short_ID conservé : 0x1)
[CAN_MANAGER_DEVICES]: full_ID: 0xC2000
[CAN_MANAGER_DEVICES]: Status device: 0xFE
[CAN_MANAGER_DEVICES]: Payload: 0x10C2000

TX_CAN: Data envoyée
[CAN_TX]: Counter: 179 | Short_ID: 0x1 | Status: 0xFD
[CAN RX TRAME 0x101]:=3=0x0=0x0=0x0 [ID:4]=0x1 [STATUS:7]=0xFD
```

---

## 9. Tester la Phase 1 (CAN Driver C2000) & Feuille de Route

### Reproduction sur Matériel (Phase 1)
Le driver applicatif CAN du `Target Node` C2000 (`src/`) est autonome et directement testable avec n'importe quelle interface USB-CAN standard (`PCAN-USB`, `SocketCAN`, `Canalyst-II`) :

1. **Configuration Matérielle (`LAUNCHXL-F2800137` + `BOOSTXL-DRV8323RS` Site 1) :**
   * Connectez votre transceiver CAN `3,3 V` sur `GPIO12` (`GPIO_12_CANA_RX`) et `GPIO13` (`GPIO_13_CANA_TX`) avec le mode analogique désactivé (`GPIO_ANALOG_DISABLED` sur ces broches AGPIO).
   * Ouvrez le projet de référence **TI MotorControl SDK — Universal Motor Control Lab (Build Level 3)** dans Code Composer Studio et insérez les blocs d'extension fournis dans `src/` aux emplacements exacts suivants :
     * `src/can_hal_snippet.h` $\rightarrow$ à coller dans **`hal.h`** (prototype `HAL_setupCAN`, ligne ~3468)
     * `src/can_hal_snippet.c` $\rightarrow$ à coller dans **`hal.c`** (appel `HAL_setupCAN` ligne ~347, broches `GPIO12`/`GPIO13` ligne ~1909, et implémentation en fin de fichier)
     * `src/can_main_snippet.h` $\rightarrow$ à coller dans **`sys_main.h`** (constantes, énumérations et fonctions `static inline` `Compute_Safety_CRC16` / `Pack_Float_To_Int24_Safe`)
     * `src/can_main_snippet.c` $\rightarrow$ à coller dans **`sys_main.c`** (ligne ~724), **impérativement à l'intérieur de la base de temps `1 ms`** (`HAL_CPU_TIMER0`), juste après l'effacement du drapeau `HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);` :

```c
while(systemVars.flagEnableSystem == true)
{
    systemVars.mainLoopCnt++;

    // 1ms time base
    if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
    {
        HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

        // ==================================================================
        // >>> COLLER ICI LE CONTENU DE src/can_main_snippet.c <<<
        // ==================================================================
```

   * Compilez et flashez la cible (`TMS320F2800137`).


2. **Paramétrage Bus :**
   * Configurez votre bus CAN à **`500 kbps`** (trames standard 11 bits, 8 octets, avec deux résistances de terminaison de **`120 Ω`**).
3. **Échanges Protocole :**
   * Émettez vos trames de commande sur l'ID **`0x555`** (et écoutez les réponses sur **`0x100`**, **`0x101`**, **`0x102`**) selon la spécification ci-dessus.

### Feuille de Route (*Roadmap*)
* **`v1.0` — Phase 1 (Actuel — Open Source Apache 2.0) :** Publication du **CAN Driver & Protocole (`Target Node` TI C2000)** audité MISRA C:2012 / EN 50128, spécification complète du protocole et preuves de validation matérielle.
* **`v1.1` — Phase 2 (Prochainement — Binaire Évaluation) :** Mise à disposition de la version compilée d'évaluation **`CAN Server STM32 Lite`** (`NUCLEO-F446RE`) dans les *Releases* GitHub pour permettre le routage UART ↔ CAN prêt à l'emploi sans recompiler le serveur.
* **`v1.2` — Phase 3 (Prochainement — Outil Client HIL) :** Mise à disposition d'un utilitaire léger en ligne de commande (**`Testbench Lite CLI`**) pour piloter et valider rapidement le `Target Node` C2000 depuis un PC hôte.


---

## 10. Licence & Propriété Intellectuelle

Ce projet repose sur une séparation stricte entre le driver embarqué ouvert (Phase 1), les bibliothèques tierces du constructeur et les briques d'orchestration propriétaire des Phases 2 et 3 :

* **Phase 1 — Extension CAN Driver & Protocole (`src/`, `README.md`, `misra_compliance.md`) :** Distribués sous licence **Apache License, Version 2.0** (voir les fichiers `LICENSE` et `NOTICE` à la racine du dépôt).
* **Composants SDK Texas Instruments (`driverlib`, `InstaSPIN-FOC`, structure *Universal Motor Control Lab*) :** Non redistribués dans ce dépôt ; restent régis par la licence **BSD-3-Clause** de **Texas Instruments Incorporated**.
* **Phase 2 — `CAN Server` (`STM32 NUCLEO-F446RE`) :** Le code source du coordinateur / passerelle UART-CAN reste propriétaire. Une version binaire d'évaluation (*CAN Server STM32 Lite*) sera mise à disposition dans les *Releases* GitHub sous contrat de licence binaire spécifique.
* **Phase 3 — `Test Host` (Suite d'automatisation `pytest`) :** Les scripts de test HIL et outils d'orchestration restent strictement propriétaires.

> **Avertissement Matériel (Contrôle Moteur) :** Conformément aux sections 7 et 8 de l'Apache License 2.0, ce logiciel est fourni « en l'état » sans garantie. L'auteur décline toute responsabilité en cas de dommage matériel, électrique ou corporel lié au pilotage d'un variateur de puissance ou d'un moteur brushless.