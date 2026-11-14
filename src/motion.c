#include "motion.h"
#include "pid.h"
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#define ZU DT_PATH(zephyr_user)

#define LOOP_PERIOD_MS 20     /* 50 Hz control loop */
#define CMD_TIMEOUT_MS 1000   /* stop if no new command for this long */
#define HEADING_STALE_MS 2000 /* heading older than this is ignored */
#define DUTY_MAX 0.95f        /* BTS7960 high-side bootstrap needs off time */
#define DUTY_DEADBAND 0.02f
#define MAX_STEP 0.05f /* max command change per loop (ramp) */

#define STACK_SIZE 2048
#define THREAD_PRIO 3

/* index 2*wheel = RPWM, 2*wheel+1 = LPWM, wheel order FL FR RL RR */
static const struct pwm_dt_spec pwm_ch[WHEEL_COUNT * 2] = {
    PWM_DT_SPEC_GET_BY_IDX(ZU, 0), PWM_DT_SPEC_GET_BY_IDX(ZU, 1),
    PWM_DT_SPEC_GET_BY_IDX(ZU, 2), PWM_DT_SPEC_GET_BY_IDX(ZU, 3),
    PWM_DT_SPEC_GET_BY_IDX(ZU, 4), PWM_DT_SPEC_GET_BY_IDX(ZU, 5),
    PWM_DT_SPEC_GET_BY_IDX(ZU, 6), PWM_DT_SPEC_GET_BY_IDX(ZU, 7),
};

static const struct gpio_dt_spec en_gpio = GPIO_DT_SPEC_GET(ZU, enable_gpios);

/* If a wheel spins the wrong way when driving forward, flip it here. */
static const bool invert[WHEEL_COUNT] = {false, false, false, false};

enum mode { MODE_STOP, MODE_DRIVE, MODE_HEADING };

static struct {
  enum mode mode;
  float speed;
  float turn;
  float heading_target;
  float heading_meas;
  int64_t heading_meas_ms;
  int64_t last_cmd_ms;
  bool enabled;
} st;

K_MUTEX_DEFINE(st_lock);

static pid_ctrl_t heading_pid;
static float prev_cmd[WHEEL_COUNT];
static float out_cmd[WHEEL_COUNT];

static K_THREAD_STACK_DEFINE(motion_stack, STACK_SIZE);
static struct k_thread motion_thread_data;

static float wrap180(float a) {
  while (a > 180.0f) {
    a -= 360.0f;
  }
  while (a <= -180.0f) {
    a += 360.0f;
  }
  return a;
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void apply_wheel(int w, float cmd) {
  if (invert[w]) {
    cmd = -cmd;
  }

  float duty = fabsf(cmd);

  if (duty < DUTY_DEADBAND) {
    duty = 0.0f;
  }
  if (duty > DUTY_MAX) {
    duty = DUTY_MAX;
  }

  const struct pwm_dt_spec *r = &pwm_ch[2 * w];
  const struct pwm_dt_spec *l = &pwm_ch[2 * w + 1];
  uint32_t pulse = (uint32_t)(duty * (float)r->period);

  /* switch the side that goes off first, so both are never on together */
  if (cmd >= 0.0f) {
    pwm_set_dt(l, l->period, 0);
    pwm_set_dt(r, r->period, pulse);
  } else {
    pwm_set_dt(r, r->period, 0);
    pwm_set_dt(l, l->period, pulse);
  }
}

static void control_step(float dt) {
  static enum mode prev_mode = MODE_STOP;
  enum mode mode;
  float speed, turn_in, hdg_t, hdg_m;
  int64_t hdg_ms, cmd_ms;
  bool en;

  k_mutex_lock(&st_lock, K_FOREVER);
  mode = st.mode;
  speed = st.speed;
  turn_in = st.turn;
  hdg_t = st.heading_target;
  hdg_m = st.heading_meas;
  hdg_ms = st.heading_meas_ms;
  cmd_ms = st.last_cmd_ms;
  en = st.enabled;
  k_mutex_unlock(&st_lock);

  int64_t now = k_uptime_get();
  bool active = en && mode != MODE_STOP && (now - cmd_ms) < CMD_TIMEOUT_MS;

  if (!active) {
    pid_reset(&heading_pid);
    for (int w = 0; w < WHEEL_COUNT; w++) {
      prev_cmd[w] = 0.0f;
      out_cmd[w] = 0.0f;
      apply_wheel(w, 0.0f);
    }
    prev_mode = mode;
    return;
  }

  if (mode != prev_mode) {
    pid_reset(&heading_pid);
    prev_mode = mode;
  }

  float turn = 0.0f;

  if (mode == MODE_DRIVE) {
    turn = turn_in;
  } else {
    if ((now - hdg_ms) < HEADING_STALE_MS) {
      /* heading grows clockwise, a positive error means turn right,
       * and our turn value is positive for LEFT, so negate it */
      float err = wrap180(hdg_t - hdg_m);

      turn = -pid_update_error(&heading_pid, err, dt);
    } else {
      /* no heading yet (not GPS-aligned): go straight */
      pid_reset(&heading_pid);
      turn = 0.0f;
    }
  }
  turn = clampf(turn, -1.0f, 1.0f);

  /* skid-steer mix: turning left makes the left side slower */
  float left = speed - turn;
  float right = speed + turn;
  float m = fmaxf(fabsf(left), fabsf(right));

  if (m > 1.0f) {
    left /= m; /* keep the ratio, so the turn stays the same */
    right /= m;
  }

  float target[WHEEL_COUNT];

  target[WHEEL_FL] = left;
  target[WHEEL_RL] = left;
  target[WHEEL_FR] = right;
  target[WHEEL_RR] = right;

  for (int w = 0; w < WHEEL_COUNT; w++) {
    float cmd = clampf(target[w], -1.0f, 1.0f);

    /* acceleration limit */
    cmd = clampf(cmd, prev_cmd[w] - MAX_STEP, prev_cmd[w] + MAX_STEP);
    prev_cmd[w] = cmd;
    out_cmd[w] = cmd;

    apply_wheel(w, cmd);
  }
}

static void motion_thread(void *a, void *b, void *c) {
  int64_t ref = k_uptime_get();

  while (1) {
    k_sleep(K_MSEC(LOOP_PERIOD_MS));

    float dt = (float)k_uptime_delta(&ref) / 1000.0f;

    if (dt <= 0.0f || dt > 0.2f) {
      dt = LOOP_PERIOD_MS / 1000.0f;
    }
    control_step(dt);
  }
}

int motion_init(void) {
  for (int i = 0; i < WHEEL_COUNT * 2; i++) {
    if (!pwm_is_ready_dt(&pwm_ch[i])) {
      return -ENODEV;
    }
  }
  if (!gpio_is_ready_dt(&en_gpio)) {
    return -ENODEV;
  }
  gpio_pin_configure_dt(&en_gpio, GPIO_OUTPUT_INACTIVE);

  /* starting gains: placeholders, tune on the real rover */
  pid_init(&heading_pid, 0.015f, 0.0005f, 0.002f, -1.0f, 1.0f);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    apply_wheel(w, 0.0f);
  }

  st.mode = MODE_STOP;
  st.heading_meas_ms = INT64_MIN / 2;

  k_thread_create(&motion_thread_data, motion_stack,
                  K_THREAD_STACK_SIZEOF(motion_stack), motion_thread, NULL,
                  NULL, NULL, THREAD_PRIO, 0, K_NO_WAIT);
  return 0;
}

void motion_enable(bool on) {
  k_mutex_lock(&st_lock, K_FOREVER);
  st.enabled = on;
  if (!on) {
    st.mode = MODE_STOP;
  }
  k_mutex_unlock(&st_lock);

  gpio_pin_set_dt(&en_gpio, on ? 1 : 0);
  if (!on) {
    for (int w = 0; w < WHEEL_COUNT; w++) {
      apply_wheel(w, 0.0f);
    }
  }
}

void motion_stop(void) {
  k_mutex_lock(&st_lock, K_FOREVER);
  st.mode = MODE_STOP;
  k_mutex_unlock(&st_lock);
}

void motion_set_drive(float speed, float turn) {
  k_mutex_lock(&st_lock, K_FOREVER);
  st.mode = MODE_DRIVE;
  st.speed = clampf(speed, -1.0f, 1.0f);
  st.turn = clampf(turn, -1.0f, 1.0f);
  st.last_cmd_ms = k_uptime_get();
  k_mutex_unlock(&st_lock);
}

void motion_set_heading(float speed, float heading_deg) {
  while (heading_deg < 0.0f) {
    heading_deg += 360.0f;
  }
  while (heading_deg >= 360.0f) {
    heading_deg -= 360.0f;
  }

  k_mutex_lock(&st_lock, K_FOREVER);
  st.mode = MODE_HEADING;
  st.speed = clampf(speed, -1.0f, 1.0f);
  st.heading_target = heading_deg;
  st.last_cmd_ms = k_uptime_get();
  k_mutex_unlock(&st_lock);
}

void motion_update_heading_measurement(float heading_deg) {
  k_mutex_lock(&st_lock, K_FOREVER);
  st.heading_meas = heading_deg;
  st.heading_meas_ms = k_uptime_get();
  k_mutex_unlock(&st_lock);
}

void motion_tune_heading(float kp, float ki, float kd) {
  k_mutex_lock(&st_lock, K_FOREVER);
  heading_pid.kp = kp;
  heading_pid.ki = ki;
  heading_pid.kd = kd;
  k_mutex_unlock(&st_lock);
}

void motion_get_wheel_cmd(float out[WHEEL_COUNT]) {
  for (int w = 0; w < WHEEL_COUNT; w++) {
    out[w] = out_cmd[w];
  }
}
