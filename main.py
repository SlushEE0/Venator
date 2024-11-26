from machine import Pin, PWM
import time

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


# Motor A Encoder pins using GPIO 14 and 15
encoder_a = Encoder(14, 15)

# Motor B Encoder pins using GPIO 11 and 10
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

# Set PWM frequency
pwm_a.freq(1000)
pwm_b.freq(1000)

# Push button on GPIO 22
button = Pin(22, Pin.IN, Pin.PULL_UP)

def setMotorSpeed_a(speed):
    motorSpeed = int((speed * 65535) / 100)  # Scale the speed to PWM duty cycle range (0-65535)
    
    if speed > 0:
        IN1.value(1)  # Motor A forward
        IN2.value(0)
    else:
        IN1.value(0)  # Motor A backward
        IN2.value(1)
    
    pwm_a.duty_u16(abs(motorSpeed))

def setMotorSpeed_b(speed):
    motorSpeed = int((speed * 65535) / 100)  # Scale the speed to PWM duty cycle range (0-65535)
    
    if speed > 0:
        IN3.value(1)  # Motor B forward
        IN4.value(0)
    else:
        IN3.value(0)  # Motor B backward
        IN4.value(1)
    
    pwm_b.duty_u16(abs(motorSpeed))

# Main loop to check for button press
while True:
    if button.value() == 0:  # Button pressed (assuming active low)
        print("Button pressed, starting robot...")
        setMotorSpeed_a(50)  # Set motor A speed to 50%
        setMotorSpeed_b(50)  # Set motor B speed to 50%
        
        # Debounce delay
        time.sleep(1)

    # Short delay to prevent button bouncing
    time.sleep(0.1)
