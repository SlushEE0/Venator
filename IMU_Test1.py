import time
import board
import busio
import math
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Initialize I2C
i2c = busio.I2C(board.GP5, board.GP4, frequency=400000)

# Initialize BNO08x
try:
    bno = BNO08X_I2C(i2c)
    print("BNO08x initialized successfully!")
except ValueError as e:
    print(f"Initialization error: {e}")
    while True:
        pass

# Enable features
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("Sensors enabled. Reading data...\n")

# Main loop
while True:
    try:
        # Read quaternion data
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        yaw = math.atan2(2.0 * (quat_real * quat_k + quat_i * quat_j),
                         1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k))
        yaw_degrees = math.degrees(yaw)

        print(f"Yaw: {yaw_degrees:.2f} degrees")
        time.sleep(0.5)

    except Exception as e:
        print(f"Error reading sensor data: {e}")
