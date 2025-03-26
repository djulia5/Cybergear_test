#include "driver/twai.h"  // Bibliothèque TWAI pour ESP32
#include "xiaomi_cybergear_driver.h"

// Pins pour le transceiver CAN
#define RX_PIN D7
#define TX_PIN D6

// Identifiants CAN
static const uint8_t MOTOR_HW_ID_1 = 0x7F;  // Moteur 1
static const uint8_t MOTOR_HW_ID_2 = 0x01;  // Moteur 2
static const uint8_t MOTOR_HW_ID_3 = 0x02;  // Moteur 3
static const uint8_t MASTER_CAN_ID = 0x00;  // ID du maître

// Paramètres ajustables (angles en degrés)
const float MAX_SPEED = 3.0f;  // Vitesse max en rad/s (modifiez ici !)
const float MOTOR1_ANGLE_DEG = 90.0f;  // 90° pour moteur 1
const float MOTOR2_ANGLE_DEG = 60.0f;  // 60° pour moteur 2
const float MOTOR3_ANGLE_DEG = 180.0f;  // 36° pour moteur 3 (PI/5 ≈ 36°)
const float DEG2RAD = PI / 180.0f;  // Conversion degrés → radians (nom changé)
const float MOTOR1_ANGLE = MOTOR1_ANGLE_DEG * DEG2RAD;  // Conversion en radians
const float MOTOR2_ANGLE = MOTOR2_ANGLE_DEG * DEG2RAD;  // Conversion en radians
const float MOTOR3_ANGLE = MOTOR3_ANGLE_DEG * DEG2RAD;  // Conversion en radians
const unsigned long MOVE_DELAY = (unsigned long)((max(max(MOTOR1_ANGLE, MOTOR2_ANGLE), MOTOR3_ANGLE) / MAX_SPEED) * 1000) + 50;  // Délai calculé + marge
const unsigned long PAUSE_DELAY = MOVE_DELAY / 4;  // Pause = 1/4 du temps de mouvement

// Objets pour les moteurs
XiaomiCyberGearDriver motor1(MOTOR_HW_ID_1, MASTER_CAN_ID);
XiaomiCyberGearDriver motor2(MOTOR_HW_ID_2, MASTER_CAN_ID);
XiaomiCyberGearDriver motor3(MOTOR_HW_ID_3, MASTER_CAN_ID);

bool driver_installed = false;

void setup() {
  // Initialisation du bus CAN
  motor1.init_twai(RX_PIN, TX_PIN, true);  // Avec débogage série
  delay(1000);

  // Configuration moteur 1
  motor1.init_motor(MODE_POSITION);
  motor1.set_position_kp(2.0f);
  motor1.set_limit_speed(MAX_SPEED);
  motor1.set_limit_current(5.0f);
  motor1.enable_motor();
  motor1.set_position_ref(0.0f);

  // Configuration moteur 2
  motor2.init_motor(MODE_POSITION);
  motor2.set_position_kp(2.0f);
  motor2.set_limit_speed(MAX_SPEED);
  motor2.set_limit_current(5.0f);
  motor2.enable_motor();
  motor2.set_position_ref(0.0f);

  // Configuration moteur 3
  motor3.init_motor(MODE_POSITION);
  motor3.set_position_kp(2.0f);
  motor3.set_limit_speed(MAX_SPEED);
  motor3.set_limit_current(5.0f);
  motor3.enable_motor();
  motor3.set_position_ref(0.0f);

  driver_installed = true;
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  // Aller aux positions cibles
  motor1.enable_motor();
  motor2.enable_motor();
  motor3.enable_motor();
  motor1.set_position_ref(MOTOR1_ANGLE);  // Moteur 1 à 90°
  motor2.set_position_ref(MOTOR2_ANGLE);  // Moteur 2 à 60°
  motor3.set_position_ref(MOTOR3_ANGLE);  // Moteur 3 à 36°
  delay(MOVE_DELAY);  // Délai calculé
  delay(PAUSE_DELAY); // Pause proportionnelle

  // Retour à la position initiale
  motor1.enable_motor();
  motor2.enable_motor();
  motor3.enable_motor();
  motor1.set_position_ref(0.0f);  // Moteur 1 à 0°
  motor2.set_position_ref(0.0f);  // Moteur 2 à 0°
  motor3.set_position_ref(0.0f);  // Moteur 3 à 0°
  delay(MOVE_DELAY);  // Délai calculé
  delay(PAUSE_DELAY); // Pause proportionnelle
}
