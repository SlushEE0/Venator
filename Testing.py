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
encoderaa = machine.Pin(4, machine.Pin.IN, machine.Pin.PULL_UP)# Motor A Encoder pins
encoderab = machine.Pin(5, machine.Pin.IN, machine.Pin.PULL_UP)
encoderba = machine.Pin(3, machine.Pin.IN, machine.Pin.PULL_UP)# Motor B Encoder pins
encoderbb = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)

# Variables
start = False
cpr = 1440  # Encoder Counts per Rotation
radius = 1.6  # Wheel Radius in cm 
motorA_posA = 0
motorA_posB = 0
motorB_posA = 0
motorB_posB = 0
# Encoder variables for motor A (using pins 4 and 5)
encoder_position_a = 0
last_a = 0
# Encoder variables for motor B (using pins 2 and 3)
encoder_position_b = 0
last_b = 0


# Interrupt service routine (ISR) for the encoder of motor A
def encoder_isr_a(pin):
    global encoder_position_a, last_a
    aa_val = encoderaa.value()
    ab_val = encoderab.value()
    if aa_val != last_a:  # Detects only changes
        if aa_val == ab_val:
            encoder_position_a += 1  # Clockwise
        else:
            encoder_position_a -= 1  # Counterclockwise
    last_a = aa_val

# Interrupt service routine (ISR) for the encoder of motor B
def encoder_isr_b(pin):
    global encoder_position_b, last_b
    ba_val = encoderba.value()
    bb_val = encoderbb.value()
    if ba_val != last_b:  # Detects only changes
        if ba_val == bb_val:
            encoder_position_b += 1  # Clockwise
        else:
            encoder_position_b -= 1  # Counterclockwise
    last_b = ba_val

# Attach the interrupt to the encoder pins
encoderaa.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=encoder_isr_a)
encoderab.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=encoder_isr_a)
encoderba.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=encoder_isr_b)
encoderbb.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=encoder_isr_b)


def move_drivetrain(distance):
    ideal_target_counts = abs(5) * cpr / (2 * math.pi * radius)
    actual_target_counts= round(ideal_target_counts)
    print(actual_target_counts)
    # Activate motor driver
    ENB.value(1)  # Enable Motor B
    ENA.value(1)  # Enable Motor A
    while encoder_position_a <= actual_target_counts and encoder_position_b <= actual_target_counts:
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
    ENB.value(1)
    ENA.value(1)
    IN1.value(1)
    IN2.value(1)
    IN3.value(1)
    IN4.value(1)
