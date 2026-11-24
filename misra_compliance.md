# Analyse de Conformité MISRA C:2012 & Sûreté EN 50128 (Phase 1 : CAN Driver C2000)

Ce document synthétise les choix d'architecture logicielle et les règles de codage appliquées au driver applicatif et à la pile protocolaire **CAN 2.0A (`500 kbps`)** développés pour le microcontrôleur **Texas Instruments `TMS320F2800137` (cœur C28x)** au sein du *Universal Motor Control Lab (Build Level 3)*.

---

## 1. Architecture Déterministe & Séparation des Contextes

| Exigence Sûreté (EN 50128 / MISRA C) | Implémentation Technique (`src/`) | Bénéfice |
| :--- | :--- | :--- |
| **Absence de conditions de course (*Race Conditions*)** | Exécution séquentielle en mode scrutation (*polling*) dans la base de temps matérielle **`1 ms` (`HAL_CPU_TIMER0`)** : scrutation **RX à `1 ms` (`1 kHz`)** et émission **TX cadencée à `10 ms` (`100 Hz`)**. | Aucune interruption matérielle CAN imbriquée susceptible de perturber la boucle critique de régulation moteur **InstaSPIN-FOC (`20 kHz`)**. |
| **Allocation mémoire statique uniquement** | Zéro allocation dynamique (`malloc`, `calloc`, `free` bannis). | Empreinte RAM 100 % déterministe connue à la compilation, sans risque de fragmentation. |
| **Portée minimale des variables et fonctions** | Utilisation systématique du qualificateur `static` pour toutes les variables d'état de la tâche `1 ms` et des fonctions utilitaires `static inline`. | Encapsulation stricte et prévention des effets de bord globaux (MISRA C:2012 Rule 8.7 / 8.9). |
| **Compteurs de séquence indépendants** | Trois compteurs cycliques dédiés (`tx_counter_hp`, `tx_counter_mp`, `tx_counter_lp`) avec rebouclage explicite `0..255` (`>= 255U`). | Détection fiable des pertes de trames par canal de priorité sans dépendre d'un débordement implicite de type sur architecture C28x (où `char`/`uint8_t` est stocké sur 16 bits). |

---

## 2. Synthèse de Conformité aux Règles MISRA C:2012

| Règle MISRA C:2012 | Intitulé / Objectif | Application dans le Driver CAN C2000 | Statut |
| :--- | :--- | :--- | :---: |
| **Rule 2.1 / 2.2** | Pas de code inaccessible (*unreachable*) ni de code mort | Toutes les branches conditionnelles et cas de `switch` sont atteignables et actifs. | Conforme |
| **Rule 2.7** | Pas de paramètres de fonction inutilisés | Marquage explicite `(void) handle;` dans `HAL_setupCAN()`. | Conforme |
| **Rule 7.2** | Suffixes `U` / `UL` obligatoires sur les constantes entières non signées | Toutes les constantes hexadécimales et décimales non signées portent le suffixe `U` ou `UL` (ex. `0x101UL`, `10U`, `255U`). | Conforme |
| **Rule 8.1 / 8.2** | Types explicites et prototypes complets avec noms de paramètres | Utilisation exclusive des types normalisés (`uint8_t`, `uint16_t`, `uint32_t`, `int32_t`, `float32_t`). | Conforme |
| **Rule 10.1 / 10.3 / 10.4** | Conversions implicites restreintes entre types signés/non signés et tailles différentes | Masquages explicites (`& 0x00FFU`, `& 0x00FFFFFFUL`) et transtypages explicites avant tout décalage binaire (`<<`, `>>`). | Conforme |
| **Rule 13.5** | Pas d'effets de bord dans l'opérande droit de `&&` ou `||` | Les conditions composées évaluent uniquement des variables déjà calculées. | Conforme |
| **Rule 14.3** | Expressions de contrôle non invariantes | Vérification explicite des plages et drapeaux matériels (`CAN_getNewDataFlags`, `CAN_getTxRequests`). | Conforme |
| **Rule 15.5** | Point de sortie unique (*Single Exit Point*) par fonction | `Compute_Safety_CRC16()` et `Pack_Float_To_Int24_Safe()` utilisent une variable de retour locale unique (`crc`, `packed_data`) et un seul `return` en fin de fonction. | Conforme |
| **Rule 15.7** | Clause `else` finale obligatoire dans toute chaîne `if ... else if` | Présente dans `Compute_Safety_CRC16()`, `Pack_Float_To_Int24_Safe()`, la machine à états d'identification et le rebouclage des compteurs. | Conforme |
| **Rule 16.4** | Clause `default` obligatoire dans toute instruction `switch` | Présente dans le routage des boîtes aux lettres (`canbox_process`) et le décodage des commandes (`status_cmd`). | Conforme |
| **Rule 17.7** | Valeur de retour des fonctions non-void toujours utilisée | Aucun appel de fonction retournant une valeur n'est ignoré silencieusement. | Conforme |
| **Rule 21.3** | Interdiction des fonctions d'allocation dynamique (`<stdlib.h>`) | Aucune inclusion ni appel à `malloc`/`free`. | Conforme |

---

## 3. Mécanismes de Défense & Repli Sécuritaire (EN 50128)

1. **Intégrité bout-en-bout (`Compute_Safety_CRC16`) :**
   * Polynôme **CRC-16/CCITT-FALSE (`0x1021`)**, valeur initiale **`0xFFFFU`**, calculé sur les **2 octets de poids faible de l'identifiant CAN** suivis des **6 octets utiles de la trame** (couplage d'intégrité entre l'ID de priorité et la charge utile pour prévenir les erreurs d'adressage).
   * En cas de pointeur nul (`data == NULL`) ou de longueur invalide (`length > 8U`), la fonction applique un **repli sécuritaire déterministe** en retournant `0xFFFFU`.
2. **Protection IEEE-754 & Saturation Physique (`Pack_Float_To_Int24_Safe` & `SET_SPEED`) :**
   * Rejet automatique des valeurs flottantes corrompues (`NaN`, `+Inf`, `-Inf`) et des débordements hors de la plage `[-8388.608f ; +8388.607f]` par repli sécurisé à `0UL`.
   * Saturation logicielle stricte de la consigne moteur à **`±70,0 Hz`** : toute consigne hors limites est rejetée sans modifier `motorVars_M1.speedRef_Hz` et déclenche une alerte sur la boîte haute priorité **`0x100` (`PROCESS_BOX_HP`)**.
3. **Surveillance d'État & Acquittement Matériel :**
   * Toute transition vers un défaut moteur InstaSPIN (`MOTOR_FAULT_STOP`) déclenche immédiatement l'émission de **`0xE4` (`STATUS_ERR_MOTOR_FAULT`)** sur le canal haute priorité **`0x100`**.
   * Acquittement systématique des drapeaux d'interruption DCAN (`CAN_clearInterruptStatus`) en fin de cycle RX/TX pour éviter tout blocage de boîte aux lettres.
