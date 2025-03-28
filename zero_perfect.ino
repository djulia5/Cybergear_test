#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
//#include "xiaomi_cybergear_defs.h"

// Définitions des broches CAN
#define RX_PIN D7
#define TX_PIN D6

// IDs CAN
static const uint8_t MOTOR_ID = 0x7F;
static const uint8_t MASTER_ID = 0xFD;

// Instance du driver CyberGear
XiaomiCyberGearDriver motor1(MOTOR_ID, MASTER_ID);

// Vérifier et réinitialiser le bus CAN si nécessaire
void check_and_reset_can_bus() {
    twai_status_info_t status;
    twai_get_status_info(&status);
    if (status.state != TWAI_STATE_RUNNING) {
        Serial.println("Bus CAN pas dans l’état RUNNING, réinitialisation...");
        twai_stop();
        delay(100);
        twai_start();
        delay(100);
        // Réinitialiser complètement le bus CAN
        motor1.init_twai(RX_PIN, TX_PIN, true);
    }
    Serial.print("État du bus CAN : ");
    Serial.println(status.state == TWAI_STATE_RUNNING ? "RUNNING" : "STOPPED");
}

// Fonction pour écrire un paramètre (type 18) - pour run_mode
void write_parameter(uint8_t motor_id, uint8_t master_id, uint16_t index, float value) {
    check_and_reset_can_bus();

    twai_message_t message;
    message.extd = 1;
    message.identifier = (18UL << 24) | ((uint32_t)master_id << 8) | motor_id;  // Type 18, Host CAN ID, Motor CAN ID
    message.data_length_code = 8;

    // Byte 0-1 : Index du paramètre
    message.data[0] = index & 0xFF;  // LSB
    message.data[1] = (index >> 8) & 0xFF;  // MSB
    // Byte 2-3 : Réservé (0)
    message.data[2] = 0;
    message.data[3] = 0;
    // Byte 4-7 : Valeur (float)
    uint32_t value_bits;
    memcpy(&value_bits, &value, sizeof(float));
    message.data[4] = value_bits & 0xFF;  // LSB
    message.data[5] = (value_bits >> 8) & 0xFF;
    message.data[6] = (value_bits >> 16) & 0xFF;
    message.data[7] = (value_bits >> 24) & 0xFF;  // MSB

    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        Serial.println("Commande d’écriture de paramètre envoyée.");
    } else {
        Serial.print("Erreur lors de l’envoi de la commande d’écriture : ");
        Serial.println(esp_err_to_name(ret));
    }
}

// Fonction pour définir la position actuelle comme zéro (type 6)
void set_mechanical_position_to_zero(uint8_t motor_id, uint8_t master_id) {
    check_and_reset_can_bus();

    twai_message_t message;
    message.extd = 1;
    message.identifier = (6UL << 24) | ((uint32_t)master_id << 8) | motor_id;  // 0x600FD7F
    message.data_length_code = 8;
    message.data[0] = 1;
    for (uint8_t i = 1; i < 8; i++) {
        message.data[i] = 0;
    }

    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        Serial.println("Commande CMD_SET_MECH_POSITION_TO_ZERO envoyée.");
    } else {
        Serial.print("Erreur lors de l’envoi de CMD_SET_MECH_POSITION_TO_ZERO : ");
        Serial.println(esp_err_to_name(ret));
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Étape 1 : Initialisation du bus CAN
    Serial.println("Initialisation du bus CAN...");
    if (motor1.init_twai(RX_PIN, TX_PIN, true) != ESP_OK) {
        Serial.println("Erreur d’initialisation du bus CAN !");
        while (1) delay(1000);
    }
    delay(1000);

    // Étape 2 : Initialisation standard du moteur (comme dans le premier code)
    Serial.println("Initialisation standard du moteur...");
    motor1.init_motor(MODE_POSITION);
    motor1.set_position_kp(2.0f);
    motor1.set_limit_speed(5.0f);
    motor1.set_limit_current(5.0f);
    motor1.enable_motor();

    delay(1000);

    // Étape 3 : Réinitialisation du moteur pour la séquence complexe
    Serial.println("Réinitialisation du moteur...");
    motor1.stop_motor();
    delay(500);

    // Étape 7 : Définir la position actuelle comme zéro
    Serial.println("Définition de la position zéro...");
    set_mechanical_position_to_zero(MOTOR_ID, MASTER_ID);
    delay(1000);

    // Étape 8 : Réactiver le moteur
    Serial.println("Réactivation du moteur...");
    motor1.enable_motor();
    delay(1000);

    // Étape 10 : Envoyer une consigne de position (par exemple, 1.0 rad)
    Serial.println("Envoi d’une consigne de position (1.0 rad)...");
    motor1.set_position_ref(1.0f);
    delay(2000);

    // Étape 11 : Ramener le moteur à la nouvelle position zéro (0.0 rad)
    Serial.println("Retour à la nouvelle position zéro (0.0 rad)...");
    motor1.set_position_ref(0.0f);
    delay(2000);
}

void loop() {
    delay(1000);  // Boucle vide
}
