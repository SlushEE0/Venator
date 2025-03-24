import machine
import time
import math
import MPU6050

# Set up I2C interface for MPU6050
i2c = machine.I2C(0, sda=machine.Pin(4), scl=machine.Pin(5))
other=machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3))
mpu = MPU6050.MPU6050(i2c)
mpu.wake()

# Gyro sensitivity for ±250°/s range
GYRO_SENSITIVITY = 1

# ====== Gyro Calibration ======
num_samples = 500
gyro_bias_z = 0

print("Calibrating gyroscope... Keep the sensor still.")

for _ in range(num_samples):
    gx, gy, gz = mpu.read_gyro_data()
    gyro_bias_z += gz

gyro_bias_z /= num_samples

print(f"Calibration complete. Bias: Z={gyro_bias_z:.2f}")

yaw = 0
prev_time = time.ticks_ms()

while True:
    gyro_x, gyro_y, gyro_z = mpu.read_gyro_data()
    curr_time = time.ticks_ms()
    dt = (curr_time - prev_time) / 1000  # Convert ms to seconds

    # Subtract bias (drift correction)
    gyro_z -= gyro_bias_z  

    # Convert gyro rate to angle (integration)
    yaw += (gyro_z / GYRO_SENSITIVITY) * dt

    # Keep yaw within 0-360° range

    print(f"Yaw: {yaw:.2f}° | Gyro Z: {(gyro_z):.2f}°/s")

    prev_time = curr_time
