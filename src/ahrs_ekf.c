#include "ahrs_ekf.h"
#include <math.h>
#include <string.h>

#define N 6
#define PI_F 3.14159265f

/* ---- tuning ---- */
#define Q_ANGLE 1.0e-4f    /* gyro noise, (rad/s)^2 */
#define Q_BIAS 1.0e-7f     /* gyro bias random walk */
#define R_ACC 2.5e-3f      /* (0.05 g)^2, accel unit-vector noise */
#define ACC_GATE_G 0.15f   /* ignore accel when |a| is off 1 g by more */
#define GPS_MIN_SPEED 0.6f /* m/s, below this course is unreliable */
#define GPS_MAX_RATE 0.35f /* rad/s, skip course while turning (slip) */
#define PITCH_LIMIT 1.45f  /* rad, keeps tan/1/cos finite */

static float wrap_pi(float a) {
  while (a > PI_F) {
    a -= 2.0f * PI_F;
  }
  while (a <= -PI_F) {
    a += 2.0f * PI_F;
  }
  return a;
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* C = A * B */
static void mat_mul(float C[N][N], const float A[N][N], const float B[N][N]) {
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      float s = 0.0f;

      for (int k = 0; k < N; k++) {
        s += A[i][k] * B[k][j];
      }
      C[i][j] = s;
    }
  }
}

/* C = A * B^T */
static void mat_mul_bt(float C[N][N], const float A[N][N],
                       const float B[N][N]) {
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      float s = 0.0f;

      for (int k = 0; k < N; k++) {
        s += A[i][k] * B[j][k];
      }
      C[i][j] = s;
    }
  }
}

void ekf_init(ekf_t *e, const float a[3], const float bias[3]) {
  memset(e, 0, sizeof(*e));

  float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

  if (n < 1.0e-3f) {
    n = 1.0f;
  }
  float ax = a[0] / n, ay = a[1] / n, az = a[2] / n;

  /* at rest in FRD, accel reads [sin(th), -sin(ph)cos(th), -cos(ph)cos(th)] */
  e->x[0] = atan2f(-ay, -az);
  e->x[1] = asinf(clampf(ax, -1.0f, 1.0f));
  e->x[2] = 0.0f;
  e->x[3] = bias[0];
  e->x[4] = bias[1];
  e->x[5] = bias[2];

  e->P[0][0] = 0.05f * 0.05f;
  e->P[1][1] = 0.05f * 0.05f;
  e->P[2][2] = PI_F * PI_F; /* heading completely unknown */
  e->P[3][3] = 0.01f * 0.01f;
  e->P[4][4] = 0.01f * 0.01f;
  e->P[5][5] = 0.01f * 0.01f;

  e->initialised = true;
}

void ekf_predict(ekf_t *e, const float g[3], float dt) {
  if (!e->initialised || dt <= 0.0f || dt > 0.1f) {
    return;
  }

  float phi = e->x[0];
  float th = clampf(e->x[1], -PITCH_LIMIT, PITCH_LIMIT);
  float p = g[0] - e->x[3];
  float q = g[1] - e->x[4];
  float r = g[2] - e->x[5];

  float sp = sinf(phi), cp = cosf(phi);
  float st = sinf(th), ct = cosf(th), tt = tanf(th);
  float ct2 = ct * ct;
  float qs_rc = q * sp + r * cp;
  float qc_rs = q * cp - r * sp;

  /* Euler-angle kinematics */
  float f0 = p + qs_rc * tt;
  float f1 = qc_rs;
  float f2 = qs_rc / ct;

  /* Jacobian of the process model */
  float F[N][N] = {{0}};

  F[0][0] = qc_rs * tt;
  F[0][1] = qs_rc / ct2;
  F[0][3] = -1.0f;
  F[0][4] = -sp * tt;
  F[0][5] = -cp * tt;

  F[1][0] = -qs_rc;
  F[1][4] = -cp;
  F[1][5] = sp;

  F[2][0] = qc_rs / ct;
  F[2][1] = qs_rc * st / ct2;
  F[2][4] = -sp / ct;
  F[2][5] = -cp / ct;

  /* state prediction */
  e->x[0] = wrap_pi(e->x[0] + f0 * dt);
  e->x[1] = clampf(e->x[1] + f1 * dt, -PITCH_LIMIT, PITCH_LIMIT);
  e->x[2] = wrap_pi(e->x[2] + f2 * dt);
  e->rate_r = f2;

  /* covariance prediction: P = Fd P Fd^T + Q, with Fd = I + F dt */
  float Fd[N][N];

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      Fd[i][j] = (i == j ? 1.0f : 0.0f) + F[i][j] * dt;
    }
  }

  float T[N][N], P2[N][N];

  mat_mul(T, Fd, e->P);
  mat_mul_bt(P2, T, Fd);
  memcpy(e->P, P2, sizeof(P2));

  for (int i = 0; i < 3; i++) {
    e->P[i][i] += Q_ANGLE * dt;
  }
  for (int i = 3; i < N; i++) {
    e->P[i][i] += Q_BIAS * dt;
  }
}

/* One scalar measurement update, Joseph form for numerical stability */
static void scalar_update(ekf_t *e, float innov, const float H[N], float R) {
  float PHt[N];

  for (int i = 0; i < N; i++) {
    float s = 0.0f;

    for (int j = 0; j < N; j++) {
      s += e->P[i][j] * H[j];
    }
    PHt[i] = s;
  }

  float S = R;

  for (int i = 0; i < N; i++) {
    S += H[i] * PHt[i];
  }
  if (S < 1.0e-9f) {
    return;
  }

  float K[N];

  for (int i = 0; i < N; i++) {
    K[i] = PHt[i] / S;
    e->x[i] += K[i] * innov;
  }

  float A[N][N], T[N][N], P2[N][N];

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      A[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
    }
  }
  mat_mul(T, A, e->P);
  mat_mul_bt(P2, T, A);

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      P2[i][j] += R * K[i] * K[j];
    }
  }
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      e->P[i][j] = 0.5f * (P2[i][j] + P2[j][i]); /* keep symmetric */
    }
  }

  e->x[0] = wrap_pi(e->x[0]);
  e->x[1] = clampf(e->x[1], -PITCH_LIMIT, PITCH_LIMIT);
  e->x[2] = wrap_pi(e->x[2]);
}

void ekf_update_accel(ekf_t *e, const float a[3]) {
  if (!e->initialised) {
    return;
  }

  float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float dev = fabsf(n - 1.0f);

  if (n < 1.0e-3f || dev > ACC_GATE_G) {
    return; /* accelerating or vibrating: gravity direction unreliable */
  }

  float z[3] = {a[0] / n, a[1] / n, a[2] / n};
  float R = R_ACC * (1.0f + 20.0f * dev);

  for (int k = 0; k < 3; k++) {
    float phi = e->x[0], th = e->x[1];
    float sp = sinf(phi), cp = cosf(phi);
    float st = sinf(th), ct = cosf(th);
    float h, H[N] = {0};

    if (k == 0) {
      h = st;
      H[1] = ct;
    } else if (k == 1) {
      h = -sp * ct;
      H[0] = -cp * ct;
      H[1] = sp * st;
    } else {
      h = -cp * ct;
      H[0] = sp * ct;
      H[1] = cp * st;
    }
    scalar_update(e, z[k] - h, H, R);
  }
}

void ekf_update_course(ekf_t *e, float course_rad, float speed_ms) {
  if (!e->initialised) {
    return;
  }
  if (speed_ms < GPS_MIN_SPEED || fabsf(e->rate_r) > GPS_MAX_RATE) {
    return;
  }

  /* noisier at low speed: 3 deg + 10 deg / (m/s) */
  float sigma = (3.0f + 10.0f / speed_ms) * (PI_F / 180.0f);
  float H[N] = {0, 0, 1, 0, 0, 0};
  float innov = wrap_pi(course_rad - e->x[2]);

  scalar_update(e, innov, H, sigma * sigma);
  e->yaw_aligned = true;
}

float ekf_heading_deg(const ekf_t *e) {
  float d = e->x[2] * (180.0f / PI_F);

  while (d < 0.0f) {
    d += 360.0f;
  }
  while (d >= 360.0f) {
    d -= 360.0f;
  }
  return d;
}

float ekf_roll_deg(const ekf_t *e) { return e->x[0] * (180.0f / PI_F); }

float ekf_pitch_deg(const ekf_t *e) { return e->x[1] * (180.0f / PI_F); }
