#pragma once
#include <stdbool.h>

/* Body frame: X forward, Y right, Z down.
 * State: roll, pitch, yaw (rad, yaw = heading clockwise from north),
 *        gyro bias p, q, r (rad/s). */
typedef struct {
  float x[6];
  float P[6][6];
  float rate_r; /* last heading rate, rad/s */
  bool initialised;
  bool yaw_aligned; /* true after the first GPS course update */
} ekf_t;

void ekf_init(ekf_t *e, const float accel_g[3], const float gyro_bias[3]);
void ekf_predict(ekf_t *e, const float gyro_rads[3], float dt);
void ekf_update_accel(ekf_t *e, const float accel_g[3]);
void ekf_update_course(ekf_t *e, float course_rad, float speed_ms);

float ekf_heading_deg(const ekf_t *e); /* 0..360 */
float ekf_roll_deg(const ekf_t *e);
float ekf_pitch_deg(const ekf_t *e);
