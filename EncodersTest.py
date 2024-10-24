from machine import Pin
import time

# Encoder variables for motor A (using pins 4 and 5)
encoder_position_a = 0
last_a = 0
# Encoder variables for motor B (using pins 2 and 3)
encoder_position_b = 0
last_b = 0
# Motor A Encoder pins
encoderaa = Pin(4, Pin.IN, Pin.PULL_UP)
encoderab = Pin(5, Pin.IN, Pin.PULL_UP)
# Motor B Encoder pins
encoderba = Pin(3, Pin.IN, Pin.PULL_UP)
encoderbb = Pin(2, Pin.IN, Pin.PULL_UP)
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
encoderaa.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_isr_a)
encoderab.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_isr_a)
encoderba.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_isr_b)
encoderbb.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_isr_b)

# Main loop
while True:
    print("Encoder A Position:", encoder_position_a)
    print("Encoder B Position:", encoder_position_b)
    time.sleep(0.1)  # Adjust the sleep time as needed
