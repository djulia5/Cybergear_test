#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include "xiaomi_cybergear_defs.h"

// Définitions des broches CAN (ajustées pour utiliser gpio_num_t)
#define RX_PIN GPIO_NUM_7  // Ajuste selon ta carte (D7 -> GPIO 7 par exemple)
#define TX_PIN GPIO_NUM_6  // Ajuste selon ta carte (D6 -> GPIO 6 par exemple)

// IDs CAN
static const uint8_t MOTOR_ID = 0x7F;  // ID du moteur
static const uint16_t MASTER_ID = 0x00;  // ID du maître

// Paramètres ajustables
const float HOMING_CURRENT = 5.0f;  // Courant pour le maintien

// Instance du driver CyberGear
XiaomiCyberGearDriver motor1(MOTOR_ID, MASTER_ID);

// Structure pour l’identifiant CAN étendu
typedef struct {
    uint32_t position : 5;  // Type de commande (bits 28-24)
    uint32_t host_can_id : 16;  // ID du maître (bits 23-8)
    uint32_t motor_can_id : 8;  // ID du moteur (bits 7-0)
} exCanIdInfo;

// Fonction pour définir la position actuelle comme zéro (type 6)
void set_mechanical_position_to_zero(uint8_t motor_id, uint16_t master_id) {
    twai_message_t message;
    message.extd = 1;  // Trame étendue
    exCanIdInfo can_id = {0};
    can_id.position = 6;  // Type de communication 6
    can_id.host_can_id = master_id;
    can_id.motor_can_id = motor_id;
    message.identifier = *(uint32_t*)&can_id;
    message.data_length_code = 8;
    message.data[0] = 1;  // Octet[0] = 1, comme spécifié dans le manuel
    for (uint8_t i = 1; i < 8; i++) {
        message.data[i] = 0;
    }

    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        Serial.println("Commande CMD_SET_MECH_POSITION_TO_ZERO envoyée.");
    } else {
        Serial.println("Erreur lors de l’envoi de CMD_SET_MECH_POSITION_TO_ZERO !");
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Étape 1 : Initialisation du bus CAN
    if (motor1.init_twai(RX_PIN, TX_PIN, true) != ESP_OK) {
        Serial.println("Erreur d’initialisation du bus CAN !");
        while (1) delay(1000);
    }
    delay(100);  // Attendre que le bus CAN soit prêt

    // Étape 2 : Configurer le mode du moteur (mode position)
    motor1.init_motor(MODE_POSITION);
    delay(500);

    // Étape 3 : Activer le moteur (type 3)
    motor1.enable_motor();
    Serial.println("Moteur activé.");
    delay(500);

    // Étape 4 : Définir la position actuelle comme zéro (type 6)
    set_mechanical_position_to_zero(MOTOR_ID, MASTER_ID);
    delay(500);

    // Étape 5 : Configurer les paramètres de contrôle
    motor1.set_position_kp(2.0f);      // Gain proportionnel
    motor1.set_limit_speed(5.0f);      // Limite de vitesse
    motor1.set_limit_current(HOMING_CURRENT);  // Limite de courant
    motor1.set_position_ref(0.0f);     // Demander au moteur de rester à 0 rad
    Serial.println("Consigne définie à 0 rad. Vérifie si le moteur bouge.");
}

void loop() {
    // Maintenir la consigne à 0 rad
    motor1.set_position_ref(0.0f);
    delay(100);  // Répéter toutes les 100 ms pour maintenir le couple
}
