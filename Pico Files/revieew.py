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
IN3 = Pin(19, Pin.OUT)
IN4 = Pin(20, Pin.OUT)
ENB = Pin(21, Pin.OUT)
# Set up PWM for motors
pwm_a = PWM(ENA)
pwm_b = PWM(ENB)
pwm_a.init(freq=5000, duty_ns=5000) # type: ignore
pwm_b.init(freq=5000, duty_ns=5000) # type: ignore

# PID Parameters
Kp_straight_a = 9 #not tuned
Kp_straight_b = 9 #not tuned
Kd_straight_a = 0 #not tuned
Kd_straight_b = 0 #not tuned
Kp_time= 0

Kp_distance = 0.002
Ki_distance = 0.0
Kd_distance = 0.002
Kp_turn = 0.005
Ki_turn = 0.00000
Kd_turn = 0.0
deadband_distance = 30
deadband_turn = 5

# Push button on GPIO 22
button = Pin(22, Pin.IN, Pin.PULL_UP)
turn_count=0
straight_count=0
turn_time = 2.6 # time for one turn (in seconds)
min_speed=0.29
max_turn_speed=0.3
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

def calculate_speed(traveled_distance):
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

def f(segments):
    global straight_count
    global prev_time
    global min_speed
    global motor_speed
    last_error_straight_a = 0
    last_error_straight_b=0
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
    encoder_resolution = 2400
    wheel_circumference = math.pi * wheel_diameter
    encoder_distance = (total_distance / wheel_circumference) * encoder_resolution
    traveled_distance = (encoder_a.position() + (encoder_b.position())) / 2
    distance_error = encoder_distance - traveled_distance

    while abs(distance_error) > deadband_distance:
        traveled_distance = (encoder_a.position() + (encoder_b.position())) / 2
        distance_error = encoder_distance - traveled_distance
        segments_left=distance_error * (wheel_circumference / encoder_resolution) / 25
        segments_traveled = traveled_distance * (wheel_circumference / encoder_resolution) / 25
        calculate_speed(segments_traveled)

        #going straight
        curr_time=time.time_ns()
        passed_time=(curr_time-prev_time)/1e9
        prev_time=curr_time

        encoder_a_pos = encoder_a.position()
        encoder_b_pos = encoder_b.position()

        if passed_time==0:
            speed_a=0
            speed_b=0
        else:
            speed_a=(((encoder_a_pos - last_encoder_a_position) * (wheel_circumference / encoder_resolution)) / passed_time)
            speed_b=(((encoder_b_pos - last_encoder_b_position) * (wheel_circumference / encoder_resolution)) / passed_time)
            speed_a_error=  motor_speed-speed_a
            speed_b_error= motor_speed-speed_b
        print(f"motor_speed: {motor_speed} speed_a:{speed_a} speed_b:{speed_b} speed_a_error: {speed_a_error} speed_b_error: {speed_b_error}")
        P_straight = speed_a_error * Kp_straight_a
        D_straight = (speed_a_error - last_error_straight_a) * Kd_straight_a
        correction_straight_a = P_straight +  D_straight

        P_straight = speed_b_error * Kp_straight_b
        D_straight = (speed_b_error - last_error_straight_b) * Kd_straight_b
        correction_straight_b = P_straight + D_straight

        last_error_straight_a = speed_a_error
        last_error_straight_b = speed_b_error

        speed_a = motor_speed + correction_straight_a
        speed_b = motor_speed + correction_straight_b
        # print(f"speed_a:{speed_a} speed_b:{speed_b} correction_straight_a:{correction_straight_a} correction_straight_b:{correction_straight_b}")
        # Apply minimum limits to avoid stalling
        
        # if abs(speed_a) > 0 and abs(speed_a) < min_speed:
        #     speed_a = min_speed if speed_a > 0 else -min_speed
        # if abs(speed_b) > 0 and abs(speed_b) < min_speed:
        #     speed_b = min_speed if speed_b > 0 else -min_speed

    
        set_motor_speed_a(speed_a)
        set_motor_speed_b(speed_b)
        
        # Stall detection

        if abs(encoder_a_pos - last_encoder_a_position) < MIN_STALL_THRESHOLD and abs(encoder_b_pos - last_encoder_b_position) < MIN_STALL_THRESHOLD:
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

        last_encoder_a_position = encoder_a_pos
        last_encoder_b_position = encoder_b_pos


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

    
    print(f"distance_error{distance_error}")
    time.sleep(0.5)


    
leda.value(1)  
ledb.value(1)
# Main loop to check for button press and execute commands
while True:
    if button.value() == 0:  # Button pressed (assuming active low)
        print("Button pressed, starting sequence...")
        leda.value(0)
        ledb.value(0)
        target_time = 10
        turn_num = 0
        straight_num = 8.5
        left=30
        right=30.6
        dist=25
        total_turn_time = turn_time * turn_num
        remaining_time = target_time - total_turn_time
        time_per_straight = remaining_time / straight_num
        average_speed=25/time_per_straight
        start_time=time.time_ns()
        prev_time=start_time
        f(8.5)

        print(f"run_time:{(time.time_ns()-start_time)/1e9}")
        print(average_speed)
        set_motor_speed_a(0)
        set_motor_speed_b(0)
        leda.value(1)  
        ledb.value(1)
        # Debounce delay
        time.sleep(1)   

    # Short delay to prevent button bouncing
    time.sleep(0.1)
