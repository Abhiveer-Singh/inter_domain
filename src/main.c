#include "ahrs_ekf.h"
#include "bioSensors.h"
#include "gps.h"
#include "imu.h"
#include "motion.h"
#include "tof.h"
#include <math.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#define STACK_SIZE 2048
#define IMU_STACK_SIZE 3072
#define TOF_STACK_SIZE 3072
#define PRIO 5
#define IMU_PRIO 4
#define DEG2RAD 0.01745329252f

/* GPS UART (usart6, PC6/PC7) comes from the overlay's zephyr,user node */
#define GPS_UART DEVICE_DT_GET(DT_PHANDLE(DT_PATH(zephyr_user), gps_uart))

/* 1 = run a short drive pattern for bench testing. WHEELS OFF THE GROUND! */
#define MOTION_DEMO 1

/* demo drive speed, 0.0 to 1.0 (0.5 = 50% duty) */
#define DEMO_SPEED 0.5f

struct attitude {
  float roll_deg;
  float pitch_deg;
  float heading_deg;
  bool aligned; /* heading locked to GPS course */
};

struct readings {
  struct dht11_data dht;
  struct bmp280_data bmp;
  struct uv_data uv;
  struct gps_data gps;
  struct attitude att;
  struct tof_data tof;
  float gps_hdop;
  int64_t gps_age_ms;
};

static struct readings data = {
    .dht = {NAN, NAN},
    .bmp = {NAN, NAN},
    .uv = {NAN, NAN},
    .gps = {false, 0, NAN, NAN, NAN, NAN, NAN, 0},
    .att = {NAN, NAN, NAN, false},
    .tof = {NAN},
    .gps_hdop = NAN,
    .gps_age_ms = -1,
};

K_MUTEX_DEFINE(data_lock);

static ekf_t ekf;
K_MUTEX_DEFINE(ekf_lock);

static void dht_thread(void *a, void *b, void *c) {
  struct dht11_data d;

  if (dht11_sensor_init()) {
    printk("DHT11 not ready\n");
    return;
  }
  while (1) {
    int rc = dht11_sensor_read(&d);

    if (rc == 0) {
      k_mutex_lock(&data_lock, K_FOREVER);
      data.dht = d;
      k_mutex_unlock(&data_lock);
    } else {
      printk("DHT11 read error: %d\n", rc);
    }
    k_sleep(K_SECONDS(2));
  }
}

static void bmp_thread(void *a, void *b, void *c) {
  struct bmp280_data d;

  if (bmp280_sensor_init()) {
    printk("BMP280 not ready\n");
    return;
  }
  while (1) {
    if (bmp280_sensor_read(&d) == 0) {
      k_mutex_lock(&data_lock, K_FOREVER);
      data.bmp = d;
      k_mutex_unlock(&data_lock);
    }
    k_sleep(K_MSEC(500));
  }
}

static void uv_thread(void *a, void *b, void *c) {
  struct uv_data d;

  if (uv_sensor_init()) {
    printk("UV sensor ADC setup failed\n");
    return;
  }
  while (1) {
    if (uv_sensor_read(&d) == 0) {
      k_mutex_lock(&data_lock, K_FOREVER);
      data.uv = d;
      k_mutex_unlock(&data_lock);
    }
    k_sleep(K_MSEC(200));
  }
}

static void tof_thread(void *a, void *b, void *c) {
  struct tof_data d;

  if (tof_sensor_init()) {
    printk("VL53 not ready\n");
    return;
  }
  while (1) {
    if (tof_sensor_read(&d) == 0) {
      k_mutex_lock(&data_lock, K_FOREVER);
      data.tof = d;
      k_mutex_unlock(&data_lock);
    }
    k_sleep(K_MSEC(100));
  }
}

static void gps_thread(void *a, void *b, void *c) {
  struct gps_fix f;
  struct gps_data g;
  uint32_t last_rmc = 0;

  if (gps_init(GPS_UART)) {
    printk("GPS not ready\n");
    return;
  }
  while (1) {
    /* true only for a fresh fix that passes sats/HDOP/status checks */
    bool ok = gps_get_fix(&f);

    memset(&g, 0, sizeof(g));
    g.valid = ok;
    g.satellites = f.sats;
    g.rmc_count = f.seq;
    if (ok) {
      g.lat_deg = f.lat;
      g.lon_deg = f.lon;
      g.altitude_m = f.alt_m;
      g.speed_ms = f.speed_kmh / 3.6f;
      g.course_deg = f.course_deg;
    } else {
      g.lat_deg = NAN;
      g.lon_deg = NAN;
      g.altitude_m = NAN;
      g.speed_ms = NAN;
      g.course_deg = NAN;
    }

    k_mutex_lock(&data_lock, K_FOREVER);
    data.gps = g;
    data.gps_hdop = f.hdop;
    data.gps_age_ms = f.age_ms;
    k_mutex_unlock(&data_lock);

    /* feed each new GPS report to the EKF exactly once */
    if (g.valid && g.rmc_count != last_rmc) {
      last_rmc = g.rmc_count;
      k_mutex_lock(&ekf_lock, K_FOREVER);
      ekf_update_course(&ekf, g.course_deg * DEG2RAD, g.speed_ms);
      k_mutex_unlock(&ekf_lock);
    }
    k_sleep(K_MSEC(100));
  }
}

static void imu_thread(void *a, void *b, void *c) {
  struct imu_data imu;
  float bias[3], acc[3];

  if (mpu6050_sensor_init()) {
    printk("MPU6050 not ready\n");
    return;
  }

  /* ---- startup calibration: rover must be still for ~2 s ---- */
  while (1) {
    float gsum[3] = {0}, asum[3] = {0};
    int n = 0;

    for (int i = 0; i < 200; i++) {
      if (mpu6050_sensor_read(&imu) == 0) {
        for (int k = 0; k < 3; k++) {
          gsum[k] += imu.gyro_rads[k];
          asum[k] += imu.accel_g[k];
        }
        n++;
      }
      k_sleep(K_MSEC(10));
    }
    if (n < 100) {
      printk("IMU: read errors during calibration, retrying\n");
      continue;
    }
    for (int k = 0; k < 3; k++) {
      bias[k] = gsum[k] / n;
      acc[k] = asum[k] / n;
    }
    if (fabsf(bias[0]) < 0.1f && fabsf(bias[1]) < 0.1f &&
        fabsf(bias[2]) < 0.1f) {
      break;
    }
    printk("IMU: rover is moving, keep it still. Retrying\n");
  }

  k_mutex_lock(&ekf_lock, K_FOREVER);
  ekf_init(&ekf, acc, bias);
  k_mutex_unlock(&ekf_lock);
  printk("IMU: calibrated, gyro bias %.4f %.4f %.4f rad/s\n", (double)bias[0],
         (double)bias[1], (double)bias[2]);

  int64_t ref = k_uptime_get();

  while (1) {
    k_sleep(K_MSEC(10)); /* 100 Hz, matches the MPU6050 setting */

    float dt = (float)k_uptime_delta(&ref) / 1000.0f;

    if (mpu6050_sensor_read(&imu) != 0) {
      continue;
    }

    struct attitude att;

    k_mutex_lock(&ekf_lock, K_FOREVER);
    ekf_predict(&ekf, imu.gyro_rads, dt);
    ekf_update_accel(&ekf, imu.accel_g);
    att.roll_deg = ekf_roll_deg(&ekf);
    att.pitch_deg = ekf_pitch_deg(&ekf);
    att.heading_deg = ekf_heading_deg(&ekf);
    att.aligned = ekf.yaw_aligned;
    k_mutex_unlock(&ekf_lock);

    k_mutex_lock(&data_lock, K_FOREVER);
    data.att = att;
    k_mutex_unlock(&data_lock);

    /* heading only goes to the motion code once GPS has aligned it */
    if (att.aligned) {
      motion_update_heading_measurement(att.heading_deg);
    }
  }
}

K_THREAD_DEFINE(dht_tid, STACK_SIZE, dht_thread, NULL, NULL, NULL, PRIO, 0, 0);
K_THREAD_DEFINE(bmp_tid, STACK_SIZE, bmp_thread, NULL, NULL, NULL, PRIO, 0, 0);
K_THREAD_DEFINE(uv_tid, STACK_SIZE, uv_thread, NULL, NULL, NULL, PRIO, 0, 0);
K_THREAD_DEFINE(tof_tid, TOF_STACK_SIZE, tof_thread, NULL, NULL, NULL, PRIO, 0,
                0);
K_THREAD_DEFINE(gps_tid, STACK_SIZE, gps_thread, NULL, NULL, NULL, PRIO, 0, 0);
K_THREAD_DEFINE(imu_tid, IMU_STACK_SIZE, imu_thread, NULL, NULL, NULL, IMU_PRIO,
                0, 0);

int main(void) {
  int tick = 0;

  if (motion_init()) {
    printk("Motion init failed\n");
  } else {
    motion_enable(true);
  }

  while (1) {
#if MOTION_DEMO
    /* 8 s loop: forward, stop, reverse, stop */
    int phase = (int)((k_uptime_get() / 1000) % 8);

    if (phase < 3) {
      motion_set_drive(DEMO_SPEED, 0.0f); /* forward */
    } else if (phase < 4) {
      motion_set_drive(0.0f, 0.0f); /* stop briefly before reversing */
    } else if (phase < 7) {
      motion_set_drive(-DEMO_SPEED, 0.0f); /* reverse */
    } else {
      motion_set_drive(0.0f, 0.0f); /* stop */
    }
#endif

    if (++tick >= 4) { /* print every 2 s */
      struct readings d;

      tick = 0;
      k_mutex_lock(&data_lock, K_FOREVER);
      d = data;
      k_mutex_unlock(&data_lock);

      printk("----------------------------------------\n");
      printk("DHT11  : %.1f C   %.1f %%RH\n", d.dht.temp_c, d.dht.humidity);
      printk("BMP280 : %.2f C   %.2f hPa\n", d.bmp.temp_c, d.bmp.pressure_hpa);
      printk("UV     : %.3f V   UV index ~%.1f\n", d.uv.volts, d.uv.index);
      printk("ToF    : %.0f mm\n", (double)d.tof.dist_mm);
      printk("GPS    : fix=%d sats=%d lat=%.6f lon=%.6f\n", (int)d.gps.valid,
             d.gps.satellites, d.gps.lat_deg, d.gps.lon_deg);
      printk("         speed=%.2f m/s  course=%.1f deg  alt=%.1f m\n",
             (double)d.gps.speed_ms, (double)d.gps.course_deg,
             (double)d.gps.altitude_m);
      printk("         hdop=%.1f  fix age=%d ms\n", (double)d.gps_hdop,
             (int)d.gps_age_ms);
      printk("IMU    : roll=%.1f pitch=%.1f heading=%.1f deg (%s)\n",
             (double)d.att.roll_deg, (double)d.att.pitch_deg,
             (double)d.att.heading_deg,
             d.att.aligned ? "GPS-aligned" : "NOT aligned");
    }

    k_sleep(K_MSEC(500));
  }
  return 0;
}
