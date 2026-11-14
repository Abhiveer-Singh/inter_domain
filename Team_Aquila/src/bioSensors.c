#include "bioSensors.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

/* ================= DHT11 ================= */

static const struct device *const dht_dev = DEVICE_DT_GET(DT_NODELABEL(dht11));

int dht11_sensor_init(void) { return device_is_ready(dht_dev) ? 0 : -ENODEV; }

int dht11_sensor_read(struct dht11_data *out) {
  struct sensor_value t, h;
  int ret = sensor_sample_fetch(dht_dev);

  if (ret) {
    return ret;
  }
  ret = sensor_channel_get(dht_dev, SENSOR_CHAN_AMBIENT_TEMP, &t);
  if (ret) {
    return ret;
  }
  ret = sensor_channel_get(dht_dev, SENSOR_CHAN_HUMIDITY, &h);
  if (ret) {
    return ret;
  }

  out->temp_c = sensor_value_to_double(&t);
  out->humidity = sensor_value_to_double(&h);
  return 0;
}

/* ================= BMP280 ================= */

static const struct device *const bmp_dev = DEVICE_DT_GET(DT_NODELABEL(bmp280));

int bmp280_sensor_init(void) { return device_is_ready(bmp_dev) ? 0 : -ENODEV; }

int bmp280_sensor_read(struct bmp280_data *out) {
  struct sensor_value t, p;
  int ret = sensor_sample_fetch(bmp_dev);

  if (ret) {
    return ret;
  }
  ret = sensor_channel_get(bmp_dev, SENSOR_CHAN_AMBIENT_TEMP, &t);
  if (ret) {
    return ret;
  }
  ret = sensor_channel_get(bmp_dev, SENSOR_CHAN_PRESS, &p);
  if (ret) {
    return ret;
  }

  out->temp_c = sensor_value_to_double(&t);
  out->pressure_hpa = sensor_value_to_double(&p) * 10.0; /* kPa -> hPa */
  return 0;
}

/* ================= UV sensor (ADC) ================= */

/* first io-channel in zephyr,user (PA4); powered from 3.3V, no divider */
static const struct adc_dt_spec uv_adc =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

int uv_sensor_init(void) {
  if (!adc_is_ready_dt(&uv_adc)) {
    return -ENODEV;
  }
  return adc_channel_setup_dt(&uv_adc);
}

int uv_sensor_read(struct uv_data *out) {
  int16_t raw;
  struct adc_sequence seq = {
      .buffer = &raw,
      .buffer_size = sizeof(raw),
  };
  int ret = adc_sequence_init_dt(&uv_adc, &seq);

  if (ret) {
    return ret;
  }
  ret = adc_read_dt(&uv_adc, &seq);
  if (ret) {
    return ret;
  }

  int32_t mv = raw;

  ret = adc_raw_to_millivolts_dt(&uv_adc, &mv);
  if (ret) {
    return ret;
  }

  out->volts = mv / 1000.0;
  out->index = out->volts / 0.1; /* about 0.1 V per UV index */
  return 0;
}

/* ================= GPS (NMEA over UART) ================= */

#define GPS_LINE_MAX 100

struct gps_line {
  char s[GPS_LINE_MAX];
};

K_MSGQ_DEFINE(gps_q, sizeof(struct gps_line), 6, 4);

static const struct device *const gps_uart =
    DEVICE_DT_GET(DT_PHANDLE(DT_PATH(zephyr_user), gps_uart));

static char rx_line[GPS_LINE_MAX];
static size_t rx_len;
static struct gps_data gps_state;

/* UART interrupt: collects characters into lines and queues complete ones */
static void gps_uart_isr(const struct device *dev, void *user_data) {
  uint8_t c;

  uart_irq_update(dev);

  while (uart_irq_rx_ready(dev)) {
    if (uart_fifo_read(dev, &c, 1) != 1) {
      break;
    }
    if (c == '$') {
      rx_len = 0; /* a new sentence starts */
    }
    if (c == '\n' || c == '\r') {
      if (rx_len > 6) {
        rx_line[rx_len] = '\0';
        k_msgq_put(&gps_q, rx_line, K_NO_WAIT);
      }
      rx_len = 0;
    } else if (rx_len < GPS_LINE_MAX - 1) {
      rx_line[rx_len++] = (char)c;
    }
  }
}

static bool nmea_checksum_ok(const char *s) {
  if (s[0] != '$') {
    return false;
  }

  uint8_t sum = 0;
  const char *p = s + 1;

  while (*p && *p != '*') {
    sum ^= (uint8_t)*p++;
  }
  if (*p != '*') {
    return false;
  }
  return (uint8_t)strtoul(p + 1, NULL, 16) == sum;
}

static int nmea_split(char *s, char **f, int max) {
  int n = 0;

  f[n++] = s;
  for (; *s && n < max; s++) {
    if (*s == ',' || *s == '*') {
      *s = '\0';
      f[n++] = s + 1;
    }
  }
  return n;
}

/* ddmm.mmmm -> decimal degrees */
static double nmea_to_deg(const char *v, const char *hemi) {
  if (!v[0]) {
    return 0.0;
  }

  double raw = strtod(v, NULL);
  int deg = (int)(raw / 100.0);
  double d = deg + (raw - deg * 100.0) / 60.0;

  if (hemi[0] == 'S' || hemi[0] == 'W') {
    d = -d;
  }
  return d;
}

static void gps_parse_line(char *line) {
  if (!nmea_checksum_ok(line)) {
    return;
  }

  char *f[20];
  int n = nmea_split(line, f, 20);

  if (n < 8 || strlen(f[0]) < 6) {
    return;
  }

  const char *type = f[0] + 3; /* skip "$GN", "$GP", ... */

  if (strcmp(type, "RMC") == 0 && n >= 9) {
    gps_state.valid = (f[2][0] == 'A');
    if (gps_state.valid) {
      gps_state.rmc_count++;
      gps_state.lat_deg = nmea_to_deg(f[3], f[4]);
      gps_state.lon_deg = nmea_to_deg(f[5], f[6]);
      gps_state.speed_ms = (float)(strtod(f[7], NULL) * 0.514444);
      if (f[8][0]) {
        gps_state.course_deg = (float)strtod(f[8], NULL);
      }
    }
  } else if (strcmp(type, "GGA") == 0 && n >= 10) {
    gps_state.satellites = atoi(f[7]);
    gps_state.altitude_m = (float)strtod(f[9], NULL);
    if (atoi(f[6]) == 0) {
      gps_state.valid = false;
    }
  }
}

int gps_sensor_init(void) {
  if (!device_is_ready(gps_uart)) {
    return -ENODEV;
  }
  uart_irq_callback_user_data_set(gps_uart, gps_uart_isr, NULL);
  uart_irq_rx_enable(gps_uart);
  return 0;
}

int gps_sensor_read(struct gps_data *out) {
  struct gps_line ln;

  while (k_msgq_get(&gps_q, &ln, K_NO_WAIT) == 0) {
    gps_parse_line(ln.s);
  }
  *out = gps_state;
  return 0;
}
