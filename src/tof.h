#ifndef TOF_H
#define TOF_H

struct tof_data {
  float dist_mm;
};

/* returns 0 on success, -ENODEV if the sensor isn't ready */
int tof_sensor_init(void);

/* blocks for up to one timing budget (about 30-50 ms) */
int tof_sensor_read(struct tof_data *d);

#endif
