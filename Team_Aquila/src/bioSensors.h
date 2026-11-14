#pragma once
#include <stdbool.h>
#include <stdint.h>

/* ---------------- DHT11 (PA0) ---------------- */
struct dht11_data {
  double temp_c;
  double humidity; /* %RH */
};

int dht11_sensor_init(void);
int dht11_sensor_read(struct dht11_data *out);

/* ---------------- BMP280 (I2C1: PB8 SCL, PB9 SDA) ---------------- */
struct bmp280_data {
  double temp_c;
  double pressure_hpa;
};

int bmp280_sensor_init(void);
int bmp280_sensor_read(struct bmp280_data *out);

/* ---------------- CJMCU-GUVA-S12SD UV sensor (PA4) ---------------- */
struct uv_data {
  double volts;
  double index; /* approximate UV index */
};

int uv_sensor_init(void);
int uv_sensor_read(struct uv_data *out);

/* ---------------- NEO-M8N GPS (USART6: PC6 TX, PC7 RX) ---------------- */
struct gps_data {
  bool valid; /* true when the receiver has a position fix */
  int satellites;
  double lat_deg; /* + north */
  double lon_deg; /* + east */
  float altitude_m;
  float speed_ms;
  float course_deg;   /* 0..360, course over ground */
  uint32_t rmc_count; /* increments on every new valid position report */
};

int gps_sensor_init(void);
int gps_sensor_read(struct gps_data *out);
