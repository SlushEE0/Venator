from machine import I2C, Pin
import time
import math
from bno08x_i2c import *

# Define I2C pins and frequency
I2C1_SDA = Pin(4)
I2C1_SCL = Pin(5)

# Initialize I2C with higher frequency
i2c1 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=400000, timeout=200000)
print("I2C Device found at address:", i2c1.scan(), "\n")

# Initialize BNO08X sensor
bno = BNO08X_I2C(i2c1, debug=False)
print("BNO08x I2C connection: Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")



while True:
    R, T, P = bno.euler
    print("Angle: %0.1f" % (P))
