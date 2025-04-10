import smbus2
import time
import Jetson.GPIO as GPIO
from delta_IK import delta_calc_inverse

# Fonction pour convertir des coordonnées XYZ en angles et les envoyer aux servos
def set_position_xyz(servos, x, y, z):
    angles = delta_calc_inverse(x, y, z)
    if angles is None:
        print("Position non réalisable: X=", x, "Y=", y, "Z=", z)
        return

    theta1, theta2, theta3 = angles
    servos.set_target_angle(0, theta1)
    servos.set_target_angle(1, theta2)
    servos.set_target_angle(2, theta3)

    # Variables globales pour la vitesse et le délai entre les mouvements
SERVO_SPEED1 = 97  # Vitesse des servos (1 à 100)
DELAY_MOVES1 = 0.2  # Temps en secondes entre les mouvements

# Fonctions pour les positions enregistrées
def position_INIT(servos):
    set_position_xyz(servos, 0, 0, 440)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)

def position_1(servos):
    set_position_xyz(servos, -320, -120, 500)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)

def position_2(servos):
    set_position_xyz(servos, -320, -120, 620)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)

def position_3(servos):
    set_position_xyz(servos, 0, 75, 500)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)

def position_4(servos):
    set_position_xyz(servos, 0, 75, 640)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)

def position_5(servos):
    set_position_xyz(servos, 320, -125, 500)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)

def position_6(servos):
    set_position_xyz(servos, 320, -125, 620)
    servos.move_all(SERVO_SPEED1)
    time.sleep(DELAY_MOVES1)



class ServoController:
    MODE1_REG = 0x00
    PRESCALE_REG = 0xFE
    LED0_REG = 0x06

    def __init__(self, address=0x40, bus_number=1):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        self._initialize_pca9685()

    def _initialize_pca9685(self):
        self.bus.write_byte_data(self.address, self.MODE1_REG, 0x20)
        time.sleep(0.05)

        self.bus.write_byte_data(self.address, self.MODE1_REG, 0x10)
        time.sleep(0.05)

        self.bus.write_byte_data(self.address, self.PRESCALE_REG, 0x79)

        self.bus.write_byte_data(self.address, self.MODE1_REG, 0x20)
        time.sleep(0.05)

    def set_pwm(self, channel, pulse):
        self.bus.write_byte_data(self.address, self.LED0_REG + 4 * channel, 0)
        self.bus.write_byte_data(self.address, self.LED0_REG + 4 * channel + 1, 0)
        self.bus.write_byte_data(self.address, self.LED0_REG + 4 * channel + 2, pulse & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_REG + 4 * channel + 3, pulse >> 8)


class ServoGroup:
    def __init__(self, controller):
        self.controller = controller
        self.servos = []

    def add_servo(self, channel):
        servo = {
            'channel': channel,
            'current_angle': 0,
            'target_angle': 0
        }
        self.servos.append(servo)

    def _angle_to_pulse(self, angle):
        center = 307
        range_per_degree = (512 - 102) / 270.0
        return int(center + (angle * range_per_degree))

    def set_target_angle(self, channel, angle):
        for servo in self.servos:
            if servo['channel'] == channel:
                servo['target_angle'] = max(-135, min(135, angle))
                break

    def move_all(self, speed=80):
        speed = max(1, min(100, speed))

        max_movement = max(
            abs(servo['target_angle'] - servo['current_angle']) 
            for servo in self.servos
        )

        steps = max(1, int(max_movement * (101 - speed) / 10))

        for step in range(steps + 1):
            for servo in self.servos:
                progress = step / steps
                intermediate_angle = (
                    servo['current_angle'] + 
                    (servo['target_angle'] - servo['current_angle']) * progress
                )

                pulse = self._angle_to_pulse(intermediate_angle)
                self.controller.set_pwm(servo['channel'], pulse)

            time.sleep(0.005 * (101 - speed) / 100)

        for servo in self.servos:
            servo['current_angle'] = servo['target_angle']

def main():
    LED_PIN = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT)

    try:
        GPIO.output(LED_PIN, GPIO.LOW)

        controller = ServoController()
        servos = ServoGroup(controller)

        servos.add_servo(0)
        servos.add_servo(1)
        servos.add_servo(2)

        time.sleep(3)

        # Séquence de mouvements prédéfinis
        position_INIT(servos)

        position_1(servos)
        position_2(servos)
        position_1(servos)
        
        position_3(servos)
        position_4(servos)
        position_3(servos)
        
        position_5(servos)
        position_6(servos)
        position_5(servos)

        position_1(servos)
        position_2(servos)
        position_1(servos)

        position_5(servos)
        position_6(servos)
        position_5(servos)

        position_3(servos)
        position_4(servos)
        position_3(servos)

        position_INIT(servos)

         # Séquence 2

        position_1(servos)
        position_2(servos)
        position_1(servos)
        
        position_3(servos)
        position_4(servos)
        position_3(servos)
        
        position_5(servos)
        position_6(servos)
        position_5(servos)

        position_1(servos)
        position_2(servos)
        position_1(servos)

        position_5(servos)
        position_6(servos)
        position_5(servos)

        position_3(servos)
        position_4(servos)
        position_3(servos)

        position_INIT(servos)

    

        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(LED_PIN, GPIO.LOW)

    except Exception as e:
        print(f"Erreur: {str(e)}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
