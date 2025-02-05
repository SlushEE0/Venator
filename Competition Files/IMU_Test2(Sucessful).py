from machine import I2C, Pin
import time
import math
from bno08x_i2c import *

# Define I2C pins and frequency
I2C1_SDA = Pin(4)
I2C1_SCL = Pin(5)

# Initialize GPIO 3 (RST pin) and set it to low for reset
RST_PIN = Pin(3, Pin.OUT)
RST_PIN.value(0)
time.sleep(0.1)  # Wait for 100ms
RST_PIN.value(1)  # Set it high to enable the sensor

# Initialize I2C with higher frequency
i2c1 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=1000000, timeout=100000)
print("I2C Device addresses:", i2c1.scan())

# Initialize BNO08X sensor
bno = BNO08X_I2C(i2c1, debug=False)
print("BNO08x I2C connection: Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")

while True:
    R, T, P = bno.euler
    a, b, c, d = bno.quaternion
    print("Quaternion: %0.10f %0.10f" % (c, d), "Angle: %0.10f" % (P))
