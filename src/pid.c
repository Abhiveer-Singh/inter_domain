#include "pid.h"

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_min,
              float out_max) {
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
  p->out_min = out_min;
  p->out_max = out_max;
  pid_reset(p);
}

void pid_reset(pid_ctrl_t *p) {
  p->integral = 0.0f;
  p->prev_error = 0.0f;
  p->first = true;
}

float pid_update_error(pid_ctrl_t *p, float error, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }

  float deriv = p->first ? 0.0f : (error - p->prev_error) / dt;

  p->first = false;
  p->prev_error = error;

  float integral = p->integral + error * dt;
  float out = p->kp * error + p->ki * integral + p->kd * deriv;

  /* anti-windup: only keep integrating if it helps leave saturation */
  if (out > p->out_max) {
    out = p->out_max;
    if (error < 0.0f) {
      p->integral = integral;
    }
  } else if (out < p->out_min) {
    out = p->out_min;
    if (error > 0.0f) {
      p->integral = integral;
    }
  } else {
    p->integral = integral;
  }
  return out;
}

float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt) {
  return pid_update_error(p, setpoint - measurement, dt);
}
