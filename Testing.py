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
start=False

def drivetrain_forward(duration):
    # Activate motor driver
    ENB.value(1)  # Enable Motor B
    ENA.value(1)  # Enable Motor A
    # Set motor direction (example: forward)
    IN1.value(1)  # Motor A forward
    IN2.value(0)
    IN3.value(1)  # Motor B forward
    IN4.value(0)
    # Run for the specified duration
    time.sleep(duration)
    # Stop motors after the duration
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)
    ENA.value(0)
    ENB.value(0)

while start!=True:
    if button.value() == 0:
        start=True
    else:
        led.value(0)
        print("Clicked")
while start==True:
        print("True")
        led.value(not led.value())  # Toggle LED state
        time.sleep(0.2)  # Simple debounce delay
        drivetrain_forward(5)
