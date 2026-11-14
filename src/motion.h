#pragma once
#include <stdbool.h>

enum wheel { WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR, WHEEL_COUNT };

int motion_init(void);       /* starts the 50 Hz control thread */
void motion_enable(bool on); /* BTS7960 enable pin */
void motion_stop(void);

/* speed: -1..1 (forward +). turn: -1..1 (+ = left, left side slower). */
void motion_set_drive(float speed, float turn);

/* Closed-loop heading. heading in degrees, 0 = north, clockwise. */
void motion_set_heading(float speed, float heading_deg);
void motion_update_heading_measurement(float heading_deg);

void motion_tune_heading(float kp, float ki, float kd);
void motion_get_wheel_cmd(float out[WHEEL_COUNT]);
