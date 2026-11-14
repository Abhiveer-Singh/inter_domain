#include "tof.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

static const struct device *tof_dev = DEVICE_DT_GET(DT_NODELABEL(tof));

int tof_sensor_init(void) { return device_is_ready(tof_dev) ? 0 : -ENODEV; }

int tof_sensor_read(struct tof_data *d) {
  struct sensor_value v;
  int rc = sensor_sample_fetch(tof_dev);

  if (rc) {
    return rc;
  }
  rc = sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &v);
  if (rc) {
    return rc;
  }
  d->dist_mm =
      (float)sensor_value_to_double(&v) * 1000.0f; /* driver gives meters */
  return 0;
}
