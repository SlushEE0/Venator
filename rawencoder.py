from machine import Pin
import time

pin_a = Pin(14, Pin.IN)
pin_b = Pin(15, Pin.IN)

while True:
    print(f"A: {pin_a.value()}, B: {pin_b.value()}")
    time.sleep(0.1)
