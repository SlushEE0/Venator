from machine import Pin, PWM
from time import sleep, ticks_ms

# Encoder Class
class Encoder:
    def __init__(self, pin_x, pin_y, reverse=False, scale=1):
        self.reverse = reverse
        self.scale = scale
        self.pin_x = Pin(pin_x, Pin.IN, Pin.PULL_UP)
        self.pin_y = Pin(pin_y, Pin.IN, Pin.PULL_UP)
        self._pos = 0
        try:
            self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback)
            self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback)
        except TypeError:
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

# Motor Speed Control
def set_motor_speed_a(speed):
    motor_speed = int(abs(speed) * 200000)
    if speed > 0:
        IN1.value(1)
        IN2.value(0)
    elif speed < 0:
        IN1.value(0)
        IN2.value(1)
    if speed == 0:  # Brake mode
        IN1.value(1)
        IN2.value(1)
    pwm_a.duty_u16(motor_speed)

def set_motor_speed_b(speed):
    motor_speed = int(abs(speed) * 200000)
    if speed > 0:
        IN3.value(1)
        IN4.value(0)
    elif speed < 0:
        IN3.value(0)
        IN4.value(1)
    if speed == 0:  # Brake mode
        IN3.value(1)
        IN4.value(1)
    pwm_b.duty_u16(motor_speed)

ENA = Pin(16, Pin.OUT)
IN1 = Pin(17, Pin.OUT)
IN2 = Pin(18, Pin.OUT)
IN3 = Pin(20, Pin.OUT)
IN4 = Pin(19, Pin.OUT)
ENB = Pin(21, Pin.OUT)
# Set up PWM for motors
pwm_a = PWM(ENA)
pwm_b = PWM(ENB)
pwm_a.init(freq=5000, duty_u16=300000,duty_ns=5000) # type: ignore
pwm_b.init(freq=5000, duty_u16=300000, duty_ns=5000) # type: ignore

# Encoders
encoder_a = Encoder(14, 15)
encoder_b = Encoder(10, 11)

# Variables for Speed Measurement
last_time = ticks_ms()
last_pos_a = encoder_a.position()
last_pos_b = encoder_b.position()
b_speed=0.76
# Main Loop
while True:
    
        # Example: Test motor equivalence
    set_motor_speed_a(0.8)  # 30% power for Motor A
    set_motor_speed_b(b_speed)  # 15% power for Motor B
    current_time = ticks_ms()
    delta_time = current_time - last_time

    # Calculate speeds in ticks per second
    if delta_time > 0:
        pos_a = encoder_a.position()
        pos_b = encoder_b.position()
        speed_a = (pos_a - last_pos_a) / delta_time * 1000  # ticks per second
        speed_b = (pos_b - last_pos_b) / delta_time * 1000
        print(speed_a, speed_b)
        last_time = current_time
        last_pos_a = pos_a
        last_pos_b = pos_b
        if speed_a != 0:
            b_to_a_ratio = abs(speed_b / speed_a)
            print(f"Equivalent Motor B Power: {b_to_a_ratio * 100:.2f}% of Motor A Power")
        else:
            print("Motor A is stationary; cannot calculate ratio.")

    else:
        print("Error: Delta time is zero, skipping calculation.")

    sleep(0.1)


#y=-2.44262x^{3}+4.3561x^{2}-1.58162x+0.499369