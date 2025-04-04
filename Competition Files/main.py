from machine import Pin, PWM, I2C
import time
import math
from array import array
import rp2 
class Encoder:
    def __init__(self, pin_x, pin_y, reverse=False, scale=1):
        self.reverse = reverse
        self.scale = scale
        self.forward = True
        self.pin_x = Pin(pin_x, Pin.IN, Pin.PULL_UP)
        self.pin_y = Pin(pin_y, Pin.IN, Pin.PULL_UP)
        self._pos = 0
        try:
            self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback, hard=True)
            self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback, hard=True)
        except TypeError:
            self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback)
            self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback)

    def x_callback(self, pin):
        self.forward = self.pin_x.value() ^ self.pin_y.value() ^ self.reverse
        self._pos += 1 if self.forward else -1

    def y_callback(self, pin):
        self.forward = self.pin_x.value() ^ self.pin_y.value() ^ self.reverse ^ 1
        self._pos += 1 if self.forward else -1

    def position(self, value=None):
        if value is not None:
            self._pos = round(value / self.scale)
        return self._pos * self.scale

    def reset(self):
        self._pos = 0

    def value(self, value=None):
        if value is not None:
            self._pos = value
        return self._pos
# Motor A Encoder pins using GPIO 15 and 14
encoder_a = Encoder(14, 15)
encoder_b= Encoder(11,10)
encoder_a.reset()
encoder_b.reset()


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
pwm_a.init(freq=5000, duty_ns=5000) # type: ignore
pwm_b.init(freq=5000, duty_ns=5000) # type: ignore

# PID Parameters
Kp_straight = 0.01
Ki_straight = 0.0000
Kd_straight = 0.00
Kp_distance = 0.002
Ki_distance = 0.0
Kd_distance = 0.002
Kp_turn = 0.005
Ki_turn = 0.00000
Kd_turn = 0.0
Kp_time=0.1
deadband_distance = 50
deadband_turn = 5

# Push button on GPIO 22
button = Pin(22, Pin.IN, Pin.PULL_UP)
turn_count=0
straight_count=0
turn_time = 2.6 # time for one turn (in seconds)
straight_time = 0.93  # time for one straight at 50% speed (in seconds)
min_speed=0.28
max_turn_speed=0.32
MIN_STALL_THRESHOLD = 1
leda = Pin(1, Pin.OUT) 
ledb = Pin(6, Pin.OUT) 



def normalize_angle(angle):
    """Normalize an angle to the range [0, 360)."""
    return (angle+360) % 360

def set_motor_speed_a(speed):
    motor_speed_1 = int(abs(speed) * 65535)
    if speed > 0:
        IN1.value(1)
        IN2.value(0)
    elif speed < 0:
        IN1.value(0)
        IN2.value(1)
    pwm_a.duty_u16(motor_speed_1)

def set_motor_speed_b(speed):
    motor_speed_2 = int(abs(speed) * 65535)
    if speed > 0:
        IN3.value(1)
        IN4.value(0)
    elif speed < 0:
        IN3.value(0)
        IN4.value(1)
    pwm_b.duty_u16(motor_speed_2)

def calculate_speed(traveled_distance,distance_left):
    global motor_speed
    motor_speed=0
    time_elapsed=(time.time_ns()-start_time)/1e9
    time_at_destination=(turn_count*turn_time)+((straight_count+traveled_distance)*time_per_straight)
    time_error= time_at_destination-time_elapsed
    if abs(traveled_distance)<=0:
        motor_speed=average_speed
        return motor_speed
    else:      
        P_time = time_error * Kp_time 
        motor_speed = average_speed - P_time
        return motor_speed


def l():
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    leda.value(0)  
    ledb.value(1)
    global turn_count
    global min_speed
    global left
    turn_count+=1
    last_encoder_a_position = 0
    last_encoder_b_position = 0
    encoder_a = Encoder(14,15)
    encoder_b = Encoder(11,10)  
    stall_counter = 0
    recovery = 0
    motor_a_target=math.pi*left/4
    wheel_diameter=6
    encoder_resolution=1440
    wheel_circumference=math.pi*wheel_diameter
    encoder_a_ticks=(motor_a_target/wheel_circumference)*encoder_resolution
    turn_error=encoder_a_ticks-encoder_a.position()
    encoder_a.reset()
    encoder_b.reset()
    print (turn_error)
    while turn_error>deadband_turn:
        encoder_a_dist=encoder_a.position()
        turn_error=encoder_a_ticks-encoder_a_dist
        P_straight = turn_error * Kp_turn
        correction_turn = P_straight
        #stall detection
        speed_a_encoder = encoder_a.position() 
        speed_b_encoder = encoder_b.position() 
        if abs(speed_a_encoder - last_encoder_a_position) < MIN_STALL_THRESHOLD: 	
            stall_counter += 1 
            if stall_counter > 10: 
                min_speed += 0.001 # Increment minimum speed 
                speed_a = min_speed if speed_a > 0 else -min_speed 
                recovery += 1 
                print(recovery) 
                if recovery > 10: 
                    temp_speed_a=0.4 if speed_a > 0 else -0.4
                    set_motor_speed_a(temp_speed_a) # Apply a high temporary speed 
                    time.sleep(0.05) 
                    recovery = 0 # Reset recovery after applying high speed 
                else: stall_counter = 0 
        else: 
            stall_counter = 0 
        last_encoder_a_position = speed_a_encoder 
        last_encoder_b_position = speed_b_encoder
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    last_encoder_a_position = encoder_a.position()
    last_encoder_b_position = encoder_b.position()

    # Wait until both encoders stop changing
    while True:
        time.sleep(0.1)  # Prevent excessive CPU usage
        
        speed_a_encoder = encoder_a.position()
        speed_b_encoder = encoder_b.position()
        
        # Check if both encoders have stopped changing
        if speed_a_encoder == last_encoder_a_position and speed_b_encoder == last_encoder_b_position:
            break  # Robot has stopped
        
        # Update last known positions
        last_encoder_a_position = speed_a_encoder
        last_encoder_b_position = speed_b_encoder
    time.sleep(0.5)

def r():
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    leda.value(1)  
    ledb.value(0)
    global turn_count
    global min_speed
    global right
    turn_count+=1
    last_encoder_a_position = 0
    last_encoder_b_position = 0
    encoder_a = Encoder(14,15)
    encoder_b = Encoder(11,10) 
    stall_counter = 0
    recovery = 0
    motor_a_target=math.pi*right/4
    wheel_diameter=6
    encoder_resolution=1440
    wheel_circumference=math.pi*wheel_diameter
    encoder_a_ticks=(motor_a_target/wheel_circumference)*encoder_resolution
    turn_error=encoder_a_ticks+encoder_b.position()
    print (turn_error)
    encoder_a.reset()
    encoder_b.reset() 
    while turn_error>deadband_turn:
        encoder_a_dist=-1*encoder_b.position()
        print(encoder_a_dist)
        turn_error=encoder_a_ticks-encoder_a_dist
        P_straight = turn_error * Kp_turn
        correction_turn = P_straight
        speed_a = correction_turn
                # Apply minimum limits to avoid stalling
        if abs(speed_a) > 0 and abs(speed_a) < min_speed:
            speed_a = min_speed if speed_a > 0 else -min_speed

        # Optionally clamp speeds to max limits
        speed_a = max(min(speed_a, max_turn_speed), -max_turn_speed)
        set_motor_speed_a(0)
        set_motor_speed_b(speed_a)
        print(speed_a)
        #stall detection
        speed_a_encoder = encoder_a.position() 
        speed_b_encoder = encoder_b.position() 
        if abs(speed_b_encoder - last_encoder_b_position) < MIN_STALL_THRESHOLD: 	
            stall_counter += 1 
            if stall_counter > 10: 
                min_speed += 0.001 # Increment minimum speed 
                speed_a = min_speed if speed_a > 0 else -min_speed 
                recovery += 1 
                print(recovery) 
                if recovery > 10: 
                    temp_speed_a=0.4 if speed_a > 0 else -0.4
                    set_motor_speed_b(temp_speed_a) # Apply a high temporary speed 
                    time.sleep(0.05) 
                    recovery = 0 # Reset recovery after applying high speed 
                else: stall_counter = 0 
        else: 
            stall_counter = 0 
        last_encoder_a_position = speed_a_encoder 
        last_encoder_b_position = speed_b_encoder
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    last_encoder_a_position = encoder_a.position()
    last_encoder_b_position = encoder_b.position()

    # Wait until both encoders stop changing
    while True:
        time.sleep(0.1)  # Prevent excessive CPU usage
        
        speed_a_encoder = encoder_a.position()
        speed_b_encoder = encoder_b.position()
        
        # Check if both encoders have stopped changing
        if speed_a_encoder == last_encoder_a_position and speed_b_encoder == last_encoder_b_position:
            break  # Robot has stopped
        
        # Update last known positions
        last_encoder_a_position = speed_a_encoder
        last_encoder_b_position = speed_b_encoder
    time.sleep(0.5)

def f(segments):
    global dist,straight_count, min_speed, deadband_distance, Kp_straight, Ki_straight, Kd_straight, motor_speed
    error_sum_straight = 0
    last_error_straight = 0
    leda.value(0)  
    ledb.value(0)
    encoder_a = Encoder(14, 15)
    encoder_b= Encoder(10,11)
    encoder_a.reset()
    encoder_b.reset()
    last_encoder_a_position = 0
    last_encoder_b_position = 0
    stall_counter = 0
    recovery = 0
    total_distance = ((segments-1) * dist) +15.3# Total distance in cm
    wheel_diameter = 6
    encoder_resolution = 1440
    wheel_circumference = math.pi * wheel_diameter
    encoder_distance = (total_distance / wheel_circumference) * encoder_resolution
    traveled_distance = (encoder_a.position() + (encoder_b.position())) / 2
    distance_error = encoder_distance - traveled_distance

    while abs(distance_error) > deadband_distance:
        traveled_distance = (encoder_a.position() + (encoder_b.position())) / 2
        distance_error = encoder_distance - traveled_distance
        segments_left=distance_error * (wheel_circumference / encoder_resolution) / 25
        segments_traveled = traveled_distance * (wheel_circumference / encoder_resolution) / 25
        calculate_speed(segments_traveled,segments_left)
                # Calculate the error as the difference between the encoder positions
        straight_error = encoder_a.position() - encoder_b.position()
        print(straight_error, traveled_distance, encoder_distance)

        # Turn PID calculations
        P_straight = straight_error * Kp_straight
        I_straight = error_sum_straight * Ki_straight
        D_straight = (straight_error - last_error_straight) * Kd_straight
        correction_straight = P_straight + I_straight + D_straight
        # Calculate motor speeds based on correction
        speed_a = motor_speed - correction_straight
        speed_b = motor_speed + correction_straight
        # speed_b = -2.13414 * speed_b ** 3 + 3.74527 * speed_b ** 2 - 1.21031 * speed_b + 0.429783

        # Apply minimum limits to avoid stalling
        if abs(speed_a) > 0 and abs(speed_a) < min_speed:
            speed_a = min_speed if speed_a > 0 else -min_speed
        if abs(speed_b) > 0 and abs(speed_b) < min_speed:
            speed_b = min_speed if speed_b > 0 else -min_speed

        # Optionally clamp speeds to max limits
        speed_a = max(min(speed_a, 1), -1)
        speed_b = max(min(speed_b, 1), -1)
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)
        
        # Stall detection
        speed_a_encoder = encoder_a.position()
        speed_b_encoder = encoder_b.position()

        if abs(speed_a_encoder - last_encoder_a_position) < MIN_STALL_THRESHOLD and abs(speed_b_encoder - last_encoder_b_position) < MIN_STALL_THRESHOLD:
            stall_counter += 1
            if stall_counter > 10:
                min_speed += 0.001  # Increment minimum speed
                speed_a = min_speed if speed_a > 0 else -min_speed
                speed_b = min_speed if speed_b > 0 else -min_speed
                recovery += 1
                if recovery > 10:
                    temp_speed_a = 0.4 if speed_a > 0 else -0.4
                    temp_speed_b = 0.4 if speed_b > 0 else -0.4
                    set_motor_speed_a(temp_speed_a)  # Apply a high temporary speed
                    set_motor_speed_b(temp_speed_b)
                    time.sleep(0.05)
                    recovery = 0  # Reset recovery after applying high speed
        else:
            stall_counter = 0

        last_encoder_a_position = speed_a_encoder
        last_encoder_b_position = speed_b_encoder


    straight_count += segments
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    # Get initial encoder positions
    last_encoder_a_position = encoder_a.position()
    last_encoder_b_position = encoder_b.position()

    # Wait until both encoders stop changing
    while True:
        time.sleep(0.1)  # Prevent excessive CPU usage
        
        speed_a_encoder = encoder_a.position()
        speed_b_encoder = encoder_b.position()
        
        # Check if both encoders have stopped changing
        if speed_a_encoder == last_encoder_a_position and speed_b_encoder == last_encoder_b_position:
            break  # Robot has stopped
        
        # Update last known positions
        last_encoder_a_position = speed_a_encoder
        last_encoder_b_position = speed_b_encoder

    
    print(distance_error)
    time.sleep(0.5)

def b(segments):
    global dist,straight_count, min_speed, deadband_distance, Kp_straight, Ki_straight, Kd_straight, motor_speed
    error_sum_straight = 0
    last_error_straight = 0
    leda.value(0)  
    ledb.value(0)
    encoder_a = Encoder(15, 14)
    encoder_b= Encoder(11,10)
    encoder_a.reset()
    encoder_b.reset()
    last_encoder_a_position = 0
    last_encoder_b_position = 0
    stall_counter = 0
    recovery = 0
    total_distance = ((segments-1) * dist) +15.3# Total distance in cm
    wheel_diameter = 6
    encoder_resolution = 1440
    wheel_circumference = math.pi * wheel_diameter
    encoder_distance = (total_distance / wheel_circumference) * encoder_resolution
    traveled_distance = (encoder_a.position() + (encoder_b.position())) / 2
    distance_error = encoder_distance - traveled_distance

    while abs(distance_error) > deadband_distance:
        traveled_distance = (encoder_a.position() + (encoder_b.position())) / 2
        distance_error = encoder_distance - traveled_distance
        segments_left=distance_error * (wheel_circumference / encoder_resolution) / 25
        segments_traveled = traveled_distance * (wheel_circumference / encoder_resolution) / 25
        calculate_speed(segments_traveled,segments_left)
                # Calculate the error as the difference between the encoder positions
        straight_error = encoder_a.position() - encoder_b.position()
        print(straight_error, traveled_distance, encoder_distance)

        # Turn PID calculations
        P_straight = straight_error * Kp_straight
        I_straight = error_sum_straight * Ki_straight
        D_straight = (straight_error - last_error_straight) * Kd_straight
        correction_straight = P_straight + I_straight + D_straight
        speed_a = motor_speed - correction_straight
        speed_b = motor_speed + correction_straight
        # speed_b = -2.13414 * speed_b ** 3 + 3.74527 * speed_b ** 2 - 1.21031 * speed_b + 0.429783

        # Apply minimum limits to avoid stalling
        if abs(speed_a) > 0 and abs(speed_a) < min_speed:
            speed_a = min_speed if speed_a > 0 else -min_speed
        if abs(speed_b) > 0 and abs(speed_b) < min_speed:
            speed_b = min_speed if speed_b > 0 else -min_speed

        # Optionally clamp speeds to max limits
        speed_a = max(min(speed_a, 1), -1)
        speed_b = max(min(speed_b, 1), -1)
        set_motor_speed_a(-speed_a)
        set_motor_speed_b(-speed_b)
        
        # Stall detection
        speed_a_encoder = encoder_a.position()
        speed_b_encoder = encoder_b.position()

        if abs(speed_a_encoder - last_encoder_a_position) < MIN_STALL_THRESHOLD and abs(speed_b_encoder - last_encoder_b_position) < MIN_STALL_THRESHOLD:
            stall_counter += 1
            if stall_counter > 10:
                min_speed += 0.001  # Increment minimum speed
                speed_a = min_speed if speed_a > 0 else -min_speed
                speed_b = min_speed if speed_b > 0 else -min_speed
                recovery += 1
                if recovery > 10:
                    temp_speed_a = 0.4 if speed_a > 0 else -0.4
                    temp_speed_b = 0.4 if speed_b > 0 else -0.4
                    set_motor_speed_a(temp_speed_a)  # Apply a high temporary speed
                    set_motor_speed_b(temp_speed_b)
                    time.sleep(0.05)
                    recovery = 0  # Reset recovery after applying high speed
        else:
            stall_counter = 0

        last_encoder_a_position = speed_a_encoder
        last_encoder_b_position = speed_b_encoder


    straight_count += segments
    set_motor_speed_a(0)
    set_motor_speed_b(0)
    # Get initial encoder positions
    last_encoder_a_position = encoder_a.position()
    last_encoder_b_position = encoder_b.position()

    # Wait until both encoders stop changing
    while True:
        time.sleep(0.1)  # Prevent excessive CPU usage
        
        speed_a_encoder = encoder_a.position()
        speed_b_encoder = encoder_b.position()
        
        # Check if both encoders have stopped changing
        if speed_a_encoder == last_encoder_a_position and speed_b_encoder == last_encoder_b_position:
            break  # Robot has stopped
        
        # Update last known positions
        last_encoder_a_position = speed_a_encoder
        last_encoder_b_position = speed_b_encoder
    print(distance_error)
    time.sleep(0.5)


    
leda.value(1)  
ledb.value(1)
# Main loop to check for button press and execute commands
while True:
    if button.value() == 0:  # Button pressed (assuming active low)
        print("Button pressed, starting sequence...")
        leda.value(0)
        ledb.value(0)
        target_time = 72
        turn_num = 4
        straight_num = 21.3
        left=30
        right=30.6
        dist=25
        total_turn_time = turn_time * turn_num
        remaining_time = target_time - total_turn_time
        time_per_straight = remaining_time / straight_num
        global average_speed
        average_speed=0.5*(straight_time / time_per_straight)
        start_time=time.time_ns()
        l()
        print(f"time:{(time.time_ns()-start_time)/1e9}")
        print(average_speed)
        set_motor_speed_a(0)
        set_motor_speed_b(0)
        leda.value(1)  
        ledb.value(1)
        # Debounce delay
        time.sleep(1)   

    # Short delay to prevent button bouncing
    time.sleep(0.1)