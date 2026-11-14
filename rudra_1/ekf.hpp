#pragma once
#include "utils.hpp"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rover {

// Tiny dense matrix, enough for a 4-state EKF (no external dependencies).
class Mat {
public:
  Mat(int r = 0, int c = 0)
      : r_(r), c_(c), d_(static_cast<size_t>(r) * c, 0.0) {}
  static Mat identity(int n) {
    Mat m(n, n);
    for (int i = 0; i < n; ++i)
      m(i, i) = 1.0;
    return m;
  }
  int rows() const { return r_; }
  int cols() const { return c_; }
  double &operator()(int i, int j) {
    return d_[static_cast<size_t>(i) * c_ + j];
  }
  double operator()(int i, int j) const {
    return d_[static_cast<size_t>(i) * c_ + j];
  }

  Mat operator*(const Mat &o) const {
    Mat m(r_, o.c_);
    for (int i = 0; i < r_; ++i)
      for (int k = 0; k < c_; ++k) {
        double a = (*this)(i, k);
        for (int j = 0; j < o.c_; ++j)
          m(i, j) += a * o(k, j);
      }
    return m;
  }
  Mat operator+(const Mat &o) const {
    Mat m(r_, c_);
    for (size_t i = 0; i < d_.size(); ++i)
      m.d_[i] = d_[i] + o.d_[i];
    return m;
  }
  Mat operator-(const Mat &o) const {
    Mat m(r_, c_);
    for (size_t i = 0; i < d_.size(); ++i)
      m.d_[i] = d_[i] - o.d_[i];
    return m;
  }
  Mat T() const {
    Mat m(c_, r_);
    for (int i = 0; i < r_; ++i)
      for (int j = 0; j < c_; ++j)
        m(j, i) = (*this)(i, j);
    return m;
  }
  // Gauss-Jordan inverse with partial pivoting (small square matrices only).
  Mat inv() const {
    int n = r_;
    Mat a = *this, b = identity(n);
    for (int col = 0; col < n; ++col) {
      int piv = col;
      for (int i = col + 1; i < n; ++i)
        if (std::fabs(a(i, col)) > std::fabs(a(piv, col)))
          piv = i;
      if (std::fabs(a(piv, col)) < 1e-12)
        throw std::runtime_error("singular matrix");
      if (piv != col)
        for (int j = 0; j < n; ++j) {
          std::swap(a(piv, j), a(col, j));
          std::swap(b(piv, j), b(col, j));
        }
      double p = a(col, col);
      for (int j = 0; j < n; ++j) {
        a(col, j) /= p;
        b(col, j) /= p;
      }
      for (int i = 0; i < n; ++i) {
        if (i == col)
          continue;
        double f = a(i, col);
        if (f == 0.0)
          continue;
        for (int j = 0; j < n; ++j) {
          a(i, j) -= f * a(col, j);
          b(i, j) -= f * b(col, j);
        }
      }
    }
    return b;
  }

private:
  int r_, c_;
  std::vector<double> d_;
};

struct EkfConfig {
  // Process noise (variance growth per second).
  double qPos = 0.02;
  double qYaw = 0.005;
  double qVel = 0.30;
  // Measurement noise (variance).
  double rYaw = 0.02 * 0.02;   // InertialUnit yaw, rad^2
  double rSpeed = 0.10 * 0.10; // wheel odometry speed, (m/s)^2
  double rGps = 0.05 * 0.05;   // GPS position, m^2
};

// State: [x, y, yaw, v]  (planar, ground-fixed frame, yaw CCW from +x).
// Predict: gyro z (yaw rate) and body-forward acceleration (already low-pass
// filtered). Update : IMU yaw, wheel-odometry speed, and (optional) GPS
// position.
class EKF {
public:
  explicit EKF(const EkfConfig &cfg = EkfConfig())
      : cfg_(cfg), x_(4, 1), P_(Mat::identity(4)) {}

  void init(double x, double y, double yaw) {
    x_(0, 0) = x;
    x_(1, 0) = y;
    x_(2, 0) = yaw;
    x_(3, 0) = 0.0;
    P_ = Mat::identity(4);
    P_(0, 0) = P_(1, 1) = 0.5;
    P_(2, 2) = 0.1;
    P_(3, 3) = 0.5;
  }

  void predict(double yawRate, double accelFwd, double dt) {
    double x = x_(0, 0), y = x_(1, 0), th = x_(2, 0), v = x_(3, 0);
    double c = std::cos(th), s = std::sin(th);

    x_(0, 0) = x + v * c * dt + 0.5 * accelFwd * c * dt * dt;
    x_(1, 0) = y + v * s * dt + 0.5 * accelFwd * s * dt * dt;
    x_(2, 0) = wrapAngle(th + yawRate * dt);
    x_(3, 0) = v + accelFwd * dt;

    Mat F = Mat::identity(4);
    F(0, 2) = -v * s * dt;
    F(0, 3) = c * dt;
    F(1, 2) = v * c * dt;
    F(1, 3) = s * dt;

    Mat Q(4, 4);
    Q(0, 0) = Q(1, 1) = cfg_.qPos * dt;
    Q(2, 2) = cfg_.qYaw * dt;
    Q(3, 3) = cfg_.qVel * dt;

    P_ = F * P_ * F.T() + Q;
  }

  void updateYaw(double yawMeas) {
    Mat H(1, 4), z(1, 1), R(1, 1);
    H(0, 2) = 1.0;
    z(0, 0) = yawMeas;
    R(0, 0) = cfg_.rYaw;
    update(z, H, R, 0);
  }

  void updateSpeed(double vMeas) {
    Mat H(1, 4), z(1, 1), R(1, 1);
    H(0, 3) = 1.0;
    z(0, 0) = vMeas;
    R(0, 0) = cfg_.rSpeed;
    update(z, H, R, -1);
  }

  void updateGps(double gx, double gy) {
    Mat H(2, 4), z(2, 1), R(2, 2);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    z(0, 0) = gx;
    z(1, 0) = gy;
    R(0, 0) = R(1, 1) = cfg_.rGps;
    update(z, H, R, -1);
  }

  void updateSlam(double sx, double sy, double r) {
    Mat H(2, 4), z(2, 1), R(2, 2);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    z(0, 0) = sx;
    z(1, 0) = sy;
    R(0, 0) = R(1, 1) = r;
    update(z, H, R, -1);
  }

  double x() const { return x_(0, 0); }
  double y() const { return x_(1, 0); }
  double yaw() const { return x_(2, 0); }
  double v() const { return x_(3, 0); }

private:
  // yawRow: index of a residual row that is an angle and must be wrapped (-1 =
  // none).
  void update(const Mat &z, const Mat &H, const Mat &R, int yawRow) {
    Mat y = z - H * x_;
    if (yawRow >= 0)
      y(yawRow, 0) = wrapAngle(y(yawRow, 0));
    Mat S = H * P_ * H.T() + R;
    Mat K = P_ * H.T() * S.inv();
    x_ = x_ + K * y;
    x_(2, 0) = wrapAngle(x_(2, 0));
    Mat I = Mat::identity(4);
    Mat IKH = I - K * H;
    P_ = IKH * P_ * IKH.T() +
         K * R * K.T(); // Joseph form keeps P symmetric / PSD
  }

  EkfConfig cfg_;
  Mat x_;
  Mat P_;
};

} // namespace rover
