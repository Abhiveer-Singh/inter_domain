#ifndef GPS_H_
#define GPS_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/* Reject fixes worse than this (lower HDOP = better accuracy) */
#define GPS_MAX_HDOP 2.5f
/* Reject fixes with fewer satellites than this */
#define GPS_MIN_SATS 4
/* A fix older than this (ms) is reported as invalid */
#define GPS_MAX_AGE_MS 3000
/* Measurement rate in ms (NEO-6M supports down to 200 ms = 5 Hz) */
#define GPS_RATE_MS 200

struct gps_fix {
  bool valid;  /* true only when fix passes all quality checks */
  double lat;  /* decimal degrees, + = N */
  double lon;  /* decimal degrees, + = E */
  float alt_m; /* altitude above mean sea level, metres */
  float speed_kmh;
  float course_deg;
  float hdop;
  uint8_t sats;           /* satellites used */
  uint8_t quality;        /* GGA quality: 0 none, 1 GPS, 2 DGPS */
  uint8_t hour, min, sec; /* UTC */
  int64_t age_ms;         /* time since the last good GGA */
  uint32_t seq; /* increments on every new RMC report (course/speed) */
};

/* Start the GPS driver on the given UART (9600 baud NMEA). Returns 0 on
 * success. */
int gps_init(const struct device *uart);

/* Copy the latest fix. Returns true only if the fix is valid and fresh. */
bool gps_get_fix(struct gps_fix *out);

#endif /* GPS_H_ */
