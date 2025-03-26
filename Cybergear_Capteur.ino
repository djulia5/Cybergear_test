#include "driver/twai.h"  // Bibliothèque TWAI pour ESP32
#include "xiaomi_cybergear_driver.h"

// Pins pour le transceiver CAN et le capteur
#define RX_PIN D7
#define TX_PIN D6
#define LIMIT_SWITCH_PIN D5  // Broche D5 pour le capteur de fin de course

// Identifiants CAN
static const uint8_t MOTOR_HW_ID_3 = 0x02;  // Moteur 3
static const uint8_t MASTER_CAN_ID = 0x00;  // ID du maître

// Paramètres ajustables
const float HOMING_SPEED = 0.5f;  // Vitesse lente pour la recherche (rad/s)
const float HOMING_CURRENT = 5.0f;  // Courant pour la recherche et le maintien
const float ROTATION_ANGLE = 50.0f * PI / 180.0f;  // 50° en radians (≈ 0.8727 rad)

// Objet pour le moteur 3
XiaomiCyberGearDriver motor3(MOTOR_HW_ID_3, MASTER_CAN_ID);

bool driver_installed = false;
int limitSwitchState = 0;  // État du capteur de fin de course
float current_pos = 0.0f;  // Position actuelle
float zero_ref = 0.0f;  // Référence zéro après ajustement
bool motorStopped = false;  // Indicateur d’arrêt
bool rotationDone = false;  // Indicateur de rotation terminée

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialisation de la LED pour débogage
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Éteinte (logique inversée sur ESP32)

  // Initialisation du capteur
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  Serial.println("Capteur initialise sur D5 (INPUT_PULLUP).");

  // Test initial du capteur
  Serial.println("Test du capteur : actionnez le capteur pour vérifier son état.");
  for (int i = 0; i < 10; i++) {
    limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
    Serial.print("État du capteur (D5) : ");
    Serial.println(limitSwitchState == LOW ? "LOW (actif)" : "HIGH (inactif)");
    digitalWrite(LED_BUILTIN, limitSwitchState == LOW ? LOW : HIGH);
    delay(1000);
  }
  digitalWrite(LED_BUILTIN, HIGH);  // Éteindre la LED après le test

  // Initialisation du bus CAN
  if (motor3.init_twai(RX_PIN, TX_PIN, true) != ESP_OK) {
    Serial.println("Erreur d’initialisation du bus CAN !");
    while (1) delay(1000);
  }
  delay(1000);

  // Configuration du moteur 3
  motor3.init_motor(MODE_POSITION);  // Mode position pour la recherche et le maintien
  motor3.set_position_kp(2.0f);      // Gain proportionnel pour la position
  motor3.set_limit_speed(HOMING_SPEED);  // Vitesse limitée
  motor3.set_limit_current(HOMING_CURRENT);  // Courant pour la recherche et le maintien
  motor3.enable_motor();  // Activer le moteur
  Serial.println("Moteur 3 configuré en mode position.");

  Serial.println("Début de la recherche de la position de fin de course...");

  const float step = 0.05f;  // Incrément de 0.05 rad par étape
  float target_pos = 0.0f;  // Position cible pour la rotation

  // Boucle principale
  while (true) {
    limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
    Serial.print("État du capteur (D5) : ");
    Serial.println(limitSwitchState == LOW ? "LOW (actif)" : "HIGH (inactif)");

    // Allumer la LED si le capteur est activé
    digitalWrite(LED_BUILTIN, limitSwitchState == LOW ? LOW : HIGH);

    // Phase 1 : Recherche de la fin de course
    if (!motorStopped) {
      if (limitSwitchState == LOW) {  // Capteur activé
        Serial.println("Capteur détecté ! Arrêt et maintien de la position.");
        motorStopped = true;  // Marquer l’arrêt
        motor3.set_position_ref(current_pos);  // Maintenir la position actuelle
        Serial.print("Position d’arrêt du moteur 3 : ");
        Serial.print(current_pos);
        Serial.println(" rad");

        // Pas d’attente, passer directement à la rotation
        Serial.println("Rotation de 50° dans le sens antihoraire...");
        target_pos = current_pos - ROTATION_ANGLE;  // Décrémenter pour tourner antihoraire
      } else {
        current_pos += step;  // Tourner dans le sens horaire
        motor3.set_position_ref(current_pos);
        Serial.print("Position actuelle (recherche) : ");
        Serial.print(current_pos);
        Serial.println(" rad");
      }
    }
    // Phase 2 : Effectuer la rotation
    else if (motorStopped && !rotationDone) {
      if (current_pos > target_pos) {
        current_pos -= step;  // Décrémenter la position
        motor3.set_position_ref(current_pos);
        Serial.print("Position actuelle (rotation) : ");
        Serial.print(current_pos);
        Serial.println(" rad");
      } else {
        rotationDone = true;  // Marquer la rotation comme terminée
        Serial.println("Rotation terminée.");

        // Phase 3 : Définir la nouvelle position comme référence zéro
        Serial.println("Définition de la nouvelle position comme référence zéro...");
        zero_ref = current_pos;  // Sauvegarder la position comme référence zéro
        current_pos = 0.0f;  // Ajuster la position relative à 0
        motor3.set_position_ref(current_pos);
        Serial.println("Nouvelle position de référence (0 rad) établie.");
      }
    }
    // Phase 4 : Maintenir la position
    else {
      motor3.set_position_ref(current_pos);  // Maintenir la position à 0 rad
      Serial.println("Moteur 3 maintient la position à 0 rad (référence zéro).");
    }

    delay(10);  // Réduire le délai pour plus de réactivité
  }
}

void loop() {
  // Rien à faire ici, tout est géré dans setup()
}
