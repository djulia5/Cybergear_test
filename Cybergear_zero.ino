#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include "xiaomi_cybergear_defs.h"  // Inclure les définitions des commandes

#define RX_PIN D7
#define TX_PIN D6

static const uint8_t MOTOR_HW_ID_1 = 0x7F;
static const uint8_t MASTER_CAN_ID = 0x00;

// Paramètres ajustables
const float HOMING_CURRENT = 5.0f;  // Courant pour le maintien

XiaomiCyberGearDriver motor1(MOTOR_HW_ID_1, MASTER_CAN_ID);

// Fonction pour définir la position actuelle comme zéro via TWAI
void set_mechanical_position_to_zero(uint8_t can_id) {
  twai_message_t message;
  message.identifier = 0x700 + can_id;  // CAN ID pour la commande (ex. 0x77F pour MOTOR_HW_ID_1 = 0x7F)
  message.extd = 0;  // Trame standard
  message.data_length_code = 8;
  message.data[0] = CMD_SET_MECH_POSITION_TO_ZERO;  // 0x06
  message.data[1] = 0x00;
  message.data[2] = 0x00;
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;

  esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
  if (ret == ESP_OK) {
    Serial.print("Commande CMD_SET_MECH_POSITION_TO_ZERO envoyée pour le moteur avec CAN ID ");
    Serial.println(can_id, HEX);
  } else {
    Serial.println("Erreur lors de l’envoi de CMD_SET_MECH_POSITION_TO_ZERO !");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialisation du bus CAN
  if (motor1.init_twai(RX_PIN, TX_PIN, true) != ESP_OK) {
    Serial.println("Erreur d’initialisation du bus CAN !");
    while (1) delay(1000);
  }
  delay(100);  // Attendre que le bus CAN soit prêt

  // Définir la position actuelle comme zéro
  set_mechanical_position_to_zero(MOTOR_HW_ID_1);
  delay(500);  // Attendre que le moteur traite la commande

  // Configurer et activer le moteur
  motor1.init_motor(MODE_POSITION);
  motor1.set_position_kp(2.0f);
  motor1.set_limit_speed(5.0f);
  motor1.set_limit_current(HOMING_CURRENT);
  motor1.enable_motor();
  motor1.set_position_ref(0.0f);  // Demander au moteur de rester à 0 rad
  Serial.println("Moteur activé et consigne définie à 0 rad. Vérifie si le moteur bouge.");
}

void loop() {
  // Maintenir la position à 0 rad
  motor1.set_position_ref(0.0f);
  delay(100);  // Répéter toutes les 100 ms pour maintenir le couple
}
