import machine
import time

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

#Variables
start=False
cpr= 1440 #Encoder Counts per Rotation
radius= 1.6 #Wheel Radius in cm 
def read_encoders():
    motorA_pos = MotorA_EncoderA.value()
    motorA_posB = MotorA_EncoderB.value()
    motorB_pos = MotorB_EncoderA.value()
    motorB_posB = MotorB_EncoderB.value()
    
    print("Motor A Encoder A:", motorA_pos)
    print("Motor A Encoder B:", motorA_posB)
    print("Motor B Encoder A:", motorB_pos)
    print("Motor B Encoder B:", motorB_posB)
    
def drivetrain_forward(distance):
    # Activate motor driver
    ENB.value(1)  # Enable Motor B
    ENA.value(1)  # Enable Motor A
    # Set motor direction (example: forward)
    IN1.value(1)  # Motor A forward
    IN2.value(0)
    IN3.value(0)  # Motor B forward (Motor B is backwards compared to Motor A)
    IN4.value(1)
    # Run for the specified duration
    for _ in range(distance * 10):  # Adjust the multiplier as needed
        read_encoders()
        time.sleep(0.1)
    # Stop motors after the duration
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)
    ENA.value(0)
    ENB.value(0)


ENB.value(1) 
ENA.value(1)  
IN1.value(1)  
IN2.value(1)
IN3.value(1)  
IN4.value(1)
#Puts Drivetrain into Brake Mode


try:
    while not start:
        if button.value() == 1:
            start = True
            print("Clicked")
            led.value(1)
        else:
            led.value(0)

    while start:
        led.value(not led.value())
        drivetrain_forward(5)
        time.sleep(0.2)

finally:
    # Cleanup section to ensure motors stop
    ENB.value(0)
    ENA.value(0)
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)
