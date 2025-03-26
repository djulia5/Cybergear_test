#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

#define RX_PIN D7
#define TX_PIN D6

static const uint8_t MOTOR_HW_ID_1 = 0x7F;
static const uint8_t MOTOR_HW_ID_2 = 0x01;
static const uint8_t MOTOR_HW_ID_3 = 0x02;
static const uint8_t MASTER_CAN_ID = 0x00;

XiaomiCyberGearDriver motor1(MOTOR_HW_ID_1, MASTER_CAN_ID);
XiaomiCyberGearDriver motor2(MOTOR_HW_ID_2, MASTER_CAN_ID);
XiaomiCyberGearDriver motor3(MOTOR_HW_ID_3, MASTER_CAN_ID);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  motor1.init_twai(RX_PIN, TX_PIN, true);
  delay(1000);

  motor1.init_motor(MODE_POSITION);
  motor1.set_position_kp(2.0f);
  motor1.set_limit_speed(5.0f);
  motor1.set_limit_current(5.0f);
  motor1.enable_motor();
  motor1.set_position_ref(0.0f);

  motor2.init_motor(MODE_POSITION);
  motor2.set_position_kp(2.0f);
  motor2.set_limit_speed(5.0f);
  motor2.set_limit_current(5.0f);
  motor2.enable_motor();
  motor2.set_position_ref(0.0f);

  motor3.init_motor(MODE_POSITION);
  motor3.set_position_kp(2.0f);
  motor3.set_limit_speed(5.0f);
  motor3.set_limit_current(5.0f);
  motor3.enable_motor();
  motor3.set_position_ref(0.0f);

  Serial.println("Moteurs positionnés à 0.0f. Observez leur position par rapport aux vis.");
  Serial.println("Notez les écarts (en radians ou degrés) pour définir les offsets.");
}

void loop() {
  delay(1000);  // Boucle vide, moteurs restent à 0.0f
}
