#include "imu.h"
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_WHO_AM_I 0x75

#define MPU_ACCEL_LSB_PER_G 16384.0f /* +-2 g     */
#define MPU_GYRO_LSB_PER_DPS 65.5f   /* +-500 dps */
#define MPU_DEG2RAD 0.01745329252f

static const struct i2c_dt_spec mpu = I2C_DT_SPEC_GET(DT_NODELABEL(mpu6050));

int mpu6050_sensor_init(void) {
  uint8_t id;
  int ret;

  if (!i2c_is_ready_dt(&mpu)) {
    return -ENODEV;
  }
  ret = i2c_reg_read_byte_dt(&mpu, MPU_REG_WHO_AM_I, &id);
  if (ret) {
    return ret;
  }
  /* genuine parts answer 0x68, some clones use other IDs */
  if (id != 0x68 && id != 0x70 && id != 0x71 && id != 0x72 && id != 0x73 &&
      id != 0x98) {
    return -ENODEV;
  }

  i2c_reg_write_byte_dt(&mpu, MPU_REG_PWR_MGMT_1, 0x80); /* reset */
  k_msleep(100);
  i2c_reg_write_byte_dt(&mpu, MPU_REG_PWR_MGMT_1, 0x01); /* wake, PLL clock */
  k_msleep(10);

  ret = i2c_reg_write_byte_dt(&mpu, MPU_REG_CONFIG, 0x03); /* low-pass ~42 Hz */
  ret |= i2c_reg_write_byte_dt(&mpu, MPU_REG_SMPLRT_DIV,
                               0x09); /* 1 kHz / 10 = 100 Hz */
  ret |= i2c_reg_write_byte_dt(&mpu, MPU_REG_GYRO_CONFIG, 0x08); /* +-500 dps */
  ret |= i2c_reg_write_byte_dt(&mpu, MPU_REG_ACCEL_CONFIG, 0x00); /* +-2 g */
  return ret ? -EIO : 0;
}

int mpu6050_sensor_read(struct imu_data *out) {
  uint8_t b[14];
  int ret = i2c_burst_read_dt(&mpu, MPU_REG_ACCEL_XOUT_H, b, sizeof(b));

  if (ret) {
    return ret;
  }

  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  float sax = ax / MPU_ACCEL_LSB_PER_G;
  float say = ay / MPU_ACCEL_LSB_PER_G;
  float saz = az / MPU_ACCEL_LSB_PER_G;
  float sgx = gx / MPU_GYRO_LSB_PER_DPS * MPU_DEG2RAD;
  float sgy = gy / MPU_GYRO_LSB_PER_DPS * MPU_DEG2RAD;
  float sgz = gz / MPU_GYRO_LSB_PER_DPS * MPU_DEG2RAD;

  /* MOUNTING: sensor frame (X forward, Y left, Z up) -> body frame
   * (X forward, Y right, Z down). Change this if you mount it differently. */
  out->accel_g[0] = sax;
  out->accel_g[1] = -say;
  out->accel_g[2] = -saz;
  out->gyro_rads[0] = sgx;
  out->gyro_rads[1] = -sgy;
  out->gyro_rads[2] = -sgz;
  return 0;
}
