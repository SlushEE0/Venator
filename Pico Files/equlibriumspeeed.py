from machine import Pin, PWM
from time import sleep, ticks_ms
import time
# Motor and Encoder setup (same as before)
class Encoder:
    def __init__(self, pin_x, pin_y, reverse=False, scale=1):
        self.reverse = reverse
        self.scale = scale
        self.pin_x = Pin(pin_x, Pin.IN, Pin.PULL_UP)
        self.pin_y = Pin(pin_y, Pin.IN, Pin.PULL_UP)
        self._pos = 0
        self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback)
        self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback)

    def x_callback(self, pin):
        forward = self.pin_x.value() ^ self.pin_y.value() ^ self.reverse
        self._pos += 1 if forward else -1

    def y_callback(self, pin):
        forward = self.pin_x.value() ^ self.pin_y.value() ^ self.reverse ^ 1
        self._pos += 1 if forward else -1

    def position(self):
        return self._pos * self.scale

    def reset(self):
        self._pos = 0

def set_motor_speed(motor, speed, pwm, in1, in2):
    motor_speed = int(abs(speed) * 65535)
    if speed > 0:
        in1.value(1)
        in2.value(0)
    elif speed < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(1)
        in2.value(1)  # Brake mode
    pwm.duty_u16(motor_speed)

# Motor pins
ENA = Pin(16, Pin.OUT)
IN1 = Pin(17, Pin.OUT)
IN2 = Pin(18, Pin.OUT)
ENB = Pin(21, Pin.OUT)
IN3 = Pin(20, Pin.OUT)
IN4 = Pin(19, Pin.OUT)

pwm_a = PWM(ENA)
pwm_b = PWM(ENB)
pwm_a.freq(5000)
pwm_b.freq(5000)

encoder_a = Encoder(14, 15)
encoder_b = Encoder(10, 11)

# Collect data
data_points = []
power_levels = [1]

for power_a in power_levels:
    encoder_a.reset()
    encoder_b.reset()
    sleep(0.1)

    # Set Motor A speed
    set_motor_speed("A", power_a, pwm_a, IN1, IN2)

    # Find equivalent Motor B power
    for power_b in [x / 1000 for x in range(265, 1010, 5)]:  # Test Motor B power from 0 to 1.0 in steps of 0.05
        encoder_b.reset()
        encoder_a.reset()
        set_motor_speed("B", power_b, pwm_b, IN3, IN4)
        sleep(5)  # Stabilize

        # Measure speeds
        speed_a = encoder_a.position() / 5  # Ticks per second
        speed_b = encoder_b.position() / 5
        print(f"Power A: {power_a}, Power B: {power_b}, Speed A: {speed_a:.2f}, Speed B: {speed_b:.2f}")
        if abs(speed_a - speed_b) < 100:  # Check if speeds match within a tolerance
            data_points.append((power_a, power_b))
            print(f"Power A: {power_a}, Power B: {power_b}, Speed A: {speed_a:.2f}, Speed B: {speed_b:.2f}")
            break

# Stop motors
set_motor_speed("A", 0, pwm_a, IN1, IN2)
set_motor_speed("B", 0, pwm_b, IN3, IN4)
time.sleep(2)
# Print all collected data points
print("Collected Data Points:")
for point in data_points:
    print(f"Motor A Power: {point[0]}, Equivalent Motor B Power: {point[1]:.2f}")
