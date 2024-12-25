from machine import Pin, PWM, I2C
import time
import math
from bno08x_i2c import *

# Initialize I2C for IMU
I2C1_SDA = Pin(4)
I2C1_SCL = Pin(5)
i2c1 = I2C(0, sda=I2C1_SDA, scl=I2C1_SCL, freq=400000, timeout=200000)
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
IN3 = Pin(20, Pin.OUT)
IN4 = Pin(19, Pin.OUT)
ENB = Pin(21, Pin.OUT)
# Set up PWM for motors
pwm_a = PWM(ENA)
pwm_b = PWM(ENB)
pwm_a.init(freq=5000, duty_ns=5000)
pwm_b.init(freq=5000, duty_ns=5000)

# PID Parameters
Kp_straight = 0.5
Ki_straight = 0.0
Kd_straight = 0.0
Kp_distance = 0.0
Ki_distance = 0.0
Kd_distance = 0.0
Kp_turn = 0.01
Ki_turn = 0
Kd_turn = 0
deadband_distance = 10
deadband_turn = 0.07

# Push button on GPIO 22
button = Pin(22, Pin.IN, Pin.PULL_UP)

def normalize_angle(angle):
    """Normalize an angle to the range [0, 360)."""
    return (angle+360) % 360

min_speed=0.29
max_turn_speed=0.4
# Helper functions to control motor speed and direction
def set_motor_speed_a(speed):
    # motor_speed = int(min(65535, max(0, abs(speed) * 65535 / 100)))
    motor_speed_1=int(abs(speed) * 65535)
    if speed > 0:
        IN1.value(1)
        IN2.value(0)
    else:
        IN1.value(0)
        IN2.value(1)
    pwm_a.duty_u16(motor_speed_1)

def set_motor_speed_b(speed):
    # motor_speed = int(min(65535, max(0, abs(speed) * 65535 / 100)))
    motor_speed_2= int(abs(speed) * 65535)
    if speed > 0:
        IN3.value(1)
        IN4.value(0)
    else:
        IN3.value(0)
        IN4.value(1)
    pwm_b.duty_u16(motor_speed_2)

def calculate_motor_speed():
    global motor_speed
    turn_time = 3  # Time for a 90-degree turn (in seconds)
    total_turn_time = turn_time * turn_num
    remaining_time = target_time - total_turn_time
    straight_time = 0.53  # Modify this based on your specific requirements
    time_per_straight = remaining_time / straight_num
    motor_speed = 50 * (straight_time / time_per_straight)

    # Ensure motor speed does not exceed 100%
    if motor_speed > 100:
        motor_speed = 100
        
    return motor_speed

def turn_left():
    global target_yaw
    initial_yaw = normalize_angle(bno.euler[2])
    target_yaw = normalize_angle(initial_yaw + 90)

    error_sum_turn = 0
    last_error_turn = 0

    while True:
        yaw = normalize_angle(bno.euler[2])
        turn_error = target_yaw - yaw
        if abs(turn_error) < deadband_turn:
            break
        
        # Turn PID calculations
        P_turn = turn_error * Kp_turn
        I_turn = error_sum_turn * Ki_turn
        D_turn = (turn_error - last_error_turn) * Kd_turn
        correction_turn = P_turn
        
    # Calculate motor speeds based on correction
        speed_a = correction_turn
        speed_b = -correction_turn

        # Ensure motors stop at the correct target
        if abs(turn_error) < deadband_turn:
            speed_a = 0
            speed_b = 0
        else:
            # Apply minimum limits to avoid stalling
            if abs(speed_a) > 0 and abs(speed_a) < min_speed:
                speed_a = min_speed if speed_a > 0 else -min_speed
            if abs(speed_b) > 0 and abs(speed_b) < min_speed:
                speed_b = min_speed if speed_b > 0 else -min_speed

            # Optionally clamp speeds to max limits
            speed_a = max(min(speed_a, max_turn_speed), -max_turn_speed)
            speed_b = max(min(speed_b, max_turn_speed), -max_turn_speed)

        # Send speeds to motors
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)


        error_sum_turn += turn_error
        last_error_turn = turn_error
        
        print(f"Yaw: {yaw}, Target Yaw: {target_yaw}, Speed A: {speed_a}, Speed B: {speed_b}")
        #time.sleep(0.1)
    
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    return target_yaw
    time.sleep(0.2)

def turn_right():
    global target_yaw
    initial_yaw = normalize_angle(bno.euler[2])
    target_yaw = normalize_angle(initial_yaw - 90)

    error_sum_turn = 0
    last_error_turn = 0

    while True:
        yaw = normalize_angle(bno.euler[2])
        turn_error = target_yaw - yaw
        if abs(turn_error) < deadband_turn:
            break
        
        # Turn PID calculations
        P_turn = turn_error * Kp_turn
        I_turn = error_sum_turn * Ki_turn
        D_turn = (turn_error - last_error_turn) * Kd_turn
        correction_turn = P_turn
        
    # Calculate motor speeds based on correction
        speed_a = correction_turn
        speed_b = -correction_turn

        # Ensure motors stop at the correct target
        if abs(turn_error) < deadband_turn:
            speed_a = 0
            speed_b = 0
        else:
            # Apply minimum limits to avoid stalling
            if abs(speed_a) > 0 and abs(speed_a) < min_speed:
                speed_a = min_speed if speed_a > 0 else -min_speed
            if abs(speed_b) > 0 and abs(speed_b) < min_speed:
                speed_b = min_speed if speed_b > 0 else -min_speed

            # Optionally clamp speeds to max limits
            speed_a = max(min(speed_a, max_turn_speed), -max_turn_speed)
            speed_b = max(min(speed_b, max_turn_speed), -max_turn_speed)

        # Send speeds to motors
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)


        error_sum_turn += turn_error
        last_error_turn = turn_error
        
        print(f"Yaw: {yaw}, Target Yaw: {target_yaw}, Speed A: {speed_a}, Speed B: {speed_b}")
        #time.sleep(0.1)
    
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    return target_yaw
    time.sleep(0.2)

def forward():

    error_sum_straight = 0
    last_error_straight = 0

    while True:
        yaw = normalize_angle(bno.euler[2])
        straight_error = target_yaw - yaw

        # Normalize the turn error
        if straight_error > 180:
            straight_error -= 360
        elif straight_error < -180:
            straight_error += 360

        # Turn PID calculations
        P_straight = straight_error * Kp_straight
        I_straight = error_sum_straight * Ki_straight
        D_straight = (straight_error - last_error_straight) * Kd_straight
        correction_straight = P_straight + I_straight + D_straight
        
        # Calculate motor speeds based on correction
        speed_a = motor_speed + correction_straight
        speed_b = motor_speed - correction_straight

        # Apply minimum limits to avoid stalling
        if abs(speed_a) > 0 and abs(speed_a) < min_speed:
            speed_a = min_speed if speed_a > 0 else -min_speed
        if abs(speed_b) > 0 and abs(speed_b) < min_speed:
            speed_b = min_speed if speed_b > 0 else -min_speed

        # Optionally clamp speeds to max limits
        speed_a = max(min(speed_a, motor_speed), -motor_speed)
        speed_b = max(min(speed_b, motor_speed), -motor_speed)

        # Send speeds to motors
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)

        error_sum_straight += straight_error
        last_error_straight = straight_error
        
        print(f"Yaw: {yaw}, Target Yaw: {target_yaw}, Speed A: {speed_a}, Speed B: {speed_b}")
        
        # Short delay for stability
        time.sleep(0.1)


# Main loop to check for button press and execute commands
while True:
    if button.value() == 0:  # Button pressed (assuming active low)
        print("Button pressed, starting sequence...")
        # Example sequence of function calls
        # Example usage
        global target_time
        global turn_num
        global straight_num
        target_time = 85
        turn_num = 8
        straight_num = 70
        target_yaw = normalize_angle(bno.euler[2])
        motor_speed = calculate_motor_speed()
        #turn_left()
        turn_right()
        forward()
        
        # Debounce delay
        time.sleep(1)

    # Short delay to prevent button bouncing
    time.sleep(0.1)
