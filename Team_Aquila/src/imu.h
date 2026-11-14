#pragma once

/* MPU6050 IMU on I2C1, address 0x68.
 * Values are already rotated into the rover body frame:
 * X forward, Y right, Z down. Accel in g, gyro in rad/s. */
struct imu_data {
  float accel_g[3];
  float gyro_rads[3];
};

int mpu6050_sensor_init(void);
int mpu6050_sensor_read(struct imu_data *out);
