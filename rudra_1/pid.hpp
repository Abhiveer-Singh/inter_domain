#pragma once
#include "utils.hpp"

namespace rover {

// PID with filtered derivative, output clamp and conditional-integration
// anti-windup. The caller supplies the error (wrap it first for angles).
class PID {
public:
  PID(double kp, double ki, double kd, double outMin, double outMax,
      double derivTau = 0.05, double integralLimit = 1e9)
      : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax),
        tau_(derivTau), iLimit_(integralLimit) {}

  void reset() {
    integral_ = 0.0;
    dFilt_ = 0.0;
    first_ = true;
  }

  double update(double error, double dt) {
    if (dt <= 0.0)
      return lastOut_;

    double dRaw = first_ ? 0.0 : (error - prevError_) / dt;
    first_ = false;
    prevError_ = error;
    double alpha = tau_ / (tau_ + dt);
    dFilt_ = alpha * dFilt_ + (1.0 - alpha) * dRaw;

    double p = kp_ * error;
    double d = kd_ * dFilt_;
    double iCand = clamp(integral_ + ki_ * error * dt, -iLimit_, iLimit_);
    double unsat = p + iCand + d;

    // Only integrate when not saturated, or when the error pulls the output
    // back.
    bool saturatedHigh = unsat > outMax_ && error > 0.0;
    bool saturatedLow = unsat < outMin_ && error < 0.0;
    if (!saturatedHigh && !saturatedLow)
      integral_ = iCand;

    lastOut_ = clamp(p + integral_ + d, outMin_, outMax_);
    return lastOut_;
  }

private:
  double kp_, ki_, kd_, outMin_, outMax_, tau_, iLimit_;
  double integral_ = 0.0, prevError_ = 0.0, dFilt_ = 0.0, lastOut_ = 0.0;
  bool first_ = true;
};

// Limits how fast a command may change (acceleration / jerk-ish smoothing).
// maxUp: max increase per second, maxDown: max decrease per second.
class RateLimiter {
public:
  RateLimiter(double maxUp, double maxDown) : up_(maxUp), down_(maxDown) {}
  void reset(double v = 0.0) { value_ = v; }
  double value() const { return value_; }
  double update(double target, double dt) {
    value_ += clamp(target - value_, -down_ * dt, up_ * dt);
    return value_;
  }

private:
  double up_, down_;
  double value_ = 0.0;
};

} // namespace rover
