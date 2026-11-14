#pragma once
#include <stdbool.h>

typedef struct {
  float kp, ki, kd;
  float out_min, out_max;
  float integral;
  float prev_error;
  bool first;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_min,
              float out_max);
void pid_reset(pid_ctrl_t *p);

/* error = setpoint - measurement */
float pid_update_error(pid_ctrl_t *p, float error, float dt);
float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt);
