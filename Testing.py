import machine
import time
import math

# Initialize GPIO
button = machine.Pin(6, machine.Pin.IN, machine.Pin.PULL_DOWN)
led = machine.Pin(25, machine.Pin.OUT)  # Onboard LED
ENB = machine.Pin(17, machine.Pin.OUT)
IN3 = machine.Pin(19, machine.Pin.OUT)
IN2 = machine.Pin(20, machine.Pin.OUT)
IN1 = machine.Pin(21, machine.Pin.OUT)
ENA = machine.Pin(22, machine.Pin.OUT)
IN4 = machine.Pin(18, machine.Pin.OUT)
MotorB_EncoderA = machine.Pin(3, machine.Pin.IN)
MotorB_EncoderB = machine.Pin(2, machine.Pin.IN)
MotorA_EncoderA = machine.Pin(4, machine.Pin.IN)
MotorA_EncoderB = machine.Pin(5, machine.Pin.IN)

# Variables
start = False
cpr = 1440  # Encoder Counts per Rotation
radius = 1.6  # Wheel Radius in cm 

motorA_posA = 0
motorA_posB = 0
motorB_posA = 0
motorB_posB = 0


ideal_target_counts = abs(5) * cpr / (2 * math.pi * radius)
actual_target_counts= round(ideal_target_counts)
print(actual_target_counts)

def move_drivetrain(distance):
    # Activate motor driver
    ENB.value(1)  # Enable Motor B
    ENA.value(1)  # Enable Motor A

    if distance >= 0:
        # Move forward
        IN1.value(1)  # Motor A forward
        IN2.value(0)
        IN3.value(0)  # Motor B forward (Motor B is backwards compared to Motor A)
        IN4.value(1)
    else:
        # Move backward
        IN1.value(0)  # Motor A backward
        IN2.value(1)
        IN3.value(1)  # Motor B backward
        IN4.value(0)
    # Convert distance in cm to encoder counts
    ENB.value(1)
    ENA.value(1)
    IN1.value(1)
    IN2.value(1)
    IN3.value(1)
    IN4.value(1)
    
ENB.value(1)
ENA.value(1)
IN1.value(1)
IN2.value(1)
IN3.value(1)
IN4.value(1)

try:
    while not start:
        if button.value() == 1:
            start = True
            print("Clicked")
            led.value(1)
        else:
            led.value(0)

    while start:
        for i in range (0,1):
            led.value(not led.value())
            move_drivetrain(5)  # Move forward 5 cm
            time.sleep(1)  # Pause for 1 second
            move_drivetrain(-5)  # Move backward 5 cm
            time.sleep(1)  # Pause for 1 second

finally:
    # Cleanup section to ensure motors stop
    ENB.value(0)
    ENA.value(0)
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)
