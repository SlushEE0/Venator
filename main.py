from machine import Pin, PWM, I2C
import time
import math
from bno08x_i2c import *

# Initialize I2C for IMU
I2C1_SDA = Pin(4)
I2C1_SCL = Pin(5)
i2c1 = I2C(0, scl=I2C1_SDA, sda=I2C1_SCL, freq=400000, timeout=200000)
bno = BNO08X_I2C(i2c1, debug=False)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# Encoder Class
class Encoder:
    def __init__(self, pin_x, pin_y, reverse=False, scale=1):
        self.reverse = reverse
        self.scale = scale
        self.pin_x = Pin(pin_x, Pin.IN, Pin.PULL_UP)
        self.pin_y = Pin(pin_y, Pin.IN, Pin.PULL_UP)
        self._pos = 0
        self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.callback)
        self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.callback)

    def callback(self, pin):
        if self.pin_x.value() ^ self.pin_y.value() ^ self.reverse:
            self._pos += 1
        else:
            self._pos -= 1

    def get_position(self):
        return self._pos

    def reset(self):
        self._pos = 0

# Initialize encoders
encoder_a = Encoder(14, 15)
encoder_b = Encoder(11, 10)

# Motor control pins
ENA = Pin(16, Pin.OUT)
IN1 = Pin(17, Pin.OUT)
IN2 = Pin(18, Pin.OUT)
IN3 = Pin(19, Pin.OUT)
IN4 = Pin(20, Pin.OUT)
ENB = Pin(21, Pin.OUT)

# Set up PWM for motors
pwm_a = PWM(ENA)
pwm_b = PWM(ENB)
pwm_a.freq(1000)
pwm_b.freq(1000)

# PID Parameters
Kp_speed = 0.0
Ki_speed = 0.0
Kd_speed = 0.0
Kp_distance = 0.0
Ki_distance = 0.0
Kd_distance = 0.0
Kp_turn = 0.0
Ki_turn = 0.0
Kd_turn = 0.0
deadband_distance = 10
deadband_turn = 3.0

# Push button on GPIO 22
button = Pin(22, Pin.IN, Pin.PULL_UP)

# Helper functions to control motor speed and direction
def set_motor_speed_a(speed):
    motor_speed = int(speed * 65535 / 100)
    if speed > 0:
        IN1.value(1)
        IN2.value(0)
    else:
        IN1.value(0)
        IN2.value(1)
    pwm_a.duty_u16(abs(motor_speed))

def set_motor_speed_b(speed):
    motor_speed = int(speed * 65535 / 100)
    if speed > 0:
        IN3.value(1)
        IN4.value(0)
    else:
        IN3.value(0)
        IN4.value(1)
    pwm_b.duty_u16(abs(motor_speed))

def move_forward(target_distance_cm):
    wheel_circumference = 60 * math.pi
    target_counts = int((target_distance_cm / wheel_circumference) * 1440)

    error_sum_distance = 0
    last_error_distance = 0
    error_sum_turn = 0
    last_error_turn = 0

    while True:
        pos_a = encoder_a.get_position()
        pos_b = encoder_b.get_position()
        
        distance_error = target_counts - (pos_a + pos_b) / 2
        yaw = bno.euler[2]
        
        if abs(distance_error) < deadband_distance:
            break
        
        # Distance PID calculations
        P_distance = distance_error * Kp_distance
        I_distance = error_sum_distance * Ki_distance
        D_distance = (distance_error - last_error_distance) * Kd_distance
        correction_distance = P_distance + I_distance + D_distance
        
        # Turn PID calculations
        turn_error = yaw
        if abs(turn_error) < deadband_turn:
            turn_error = 0
        P_turn = turn_error * Kp_turn
        I_turn = error_sum_turn * Ki_turn
        D_turn = (turn_error - last_error_turn) * Kd_turn
        correction_turn = P_turn + I_turn + D_turn
        
        speed_a = 50 + correction_distance - correction_turn
        speed_b = 50 + correction_distance + correction_turn
        
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)

        error_sum_distance += distance_error
        last_error_distance = distance_error
        error_sum_turn += turn_error
        last_error_turn = turn_error
        
        print(f"Distance Error: {distance_error}, Yaw: {yaw}, Speed A: {speed_a}, Speed B: {speed_b}")
        time.sleep(0.1)
    
    set_motor_speed_a(0)
    set_motor_speed_b(0)

def turn_left(target_angle=90):
    initial_yaw = bno.euler[2]
    target_yaw = (initial_yaw - target_angle) % 360

    error_sum_turn = 0
    last_error_turn = 0

    while True:
        yaw = bno.euler[2]
        turn_error = target_yaw - yaw
        if abs(turn_error) < deadband_turn:
            break
        
        # Turn PID calculations
        P_turn = turn_error * Kp_turn
        I_turn = error_sum_turn * Ki_turn
        D_turn = (turn_error - last_error_turn) * Kd_turn
        correction_turn = P_turn + I_turn + D_turn
        
        speed_a = -correction_turn
        speed_b = correction_turn
        
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)

        error_sum_turn += turn_error
        last_error_turn = turn_error
        
        print(f"Yaw: {yaw}, Target Yaw: {target_yaw}, Speed A: {speed_a}, Speed B: {speed_b}")
        time.sleep(0.1)
    
    set_motor_speed_a(0)
    set_motor_speed_b(0)

def turn_right(target_angle=90):
    initial_yaw = bno.euler[2]
    target_yaw = (initial_yaw + target_angle) % 360

    error_sum_turn = 0
    last_error_turn = 0

    while True:
        yaw = bno.euler[2]
        turn_error = target_yaw - yaw
        if abs(turn_error) < deadband_turn:
            break
        
        # Turn PID calculations
        P_turn = turn_error * Kp_turn
        I_turn = error_sum_turn * Ki_turn
        D_turn = (turn_error - last_error_turn) * Kd_turn
        correction_turn = P_turn + I_turn + D_turn
        
        speed_a = correction_turn
        speed_b = -correction_turn
        
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)

        error_sum_turn += turn_error
        last_error_turn = turn_error
        
        print(f"Yaw: {yaw}, Target Yaw: {target_yaw}, Speed A: {speed_a}, Speed B: {speed_b}")
        time.sleep(0.1)
    
    set_motor_speed_a(0)
    set_motor_speed_b(0)

# Main loop to check for button press and execute commands
while True:
    if button.value() == 0:  # Button pressed (assuming active low)
        print("Button pressed, starting sequence...")
        # Example sequence of function calls
        move_forward(5)
        
        # Debounce delay
        time.sleep(1)

    # Short delay to prevent button bouncing
    time.sleep(0.1)
