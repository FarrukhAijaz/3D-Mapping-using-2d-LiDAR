from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import math
from time import sleep

# Constants
MIN_PULSE_WIDTH = 500 / 1_000_000  # For MG996R
MAX_PULSE_WIDTH = 2500 / 1_000_000  # For MG996R
SERVO_PIN = 12

class ServoControl:

    def __init__(self):
        # Initialize servo
        factory = PiGPIOFactory()
        self.servo = Servo(SERVO_PIN, min_pulse_width=MIN_PULSE_WIDTH, max_pulse_width=MAX_PULSE_WIDTH, pin_factory=factory)

    def set_servo_angle(self, vertical_angle_deg):
        vertical_angle_rad = math.radians(vertical_angle_deg)
        self.servo.value = math.sin(vertical_angle_rad)
        sleep(0.1)  # Allow some time for the servo to move

if __name__ == '__main__':
    servo_control = ServoControl()
    servo_control.set_servo_angle(0)
