#pragma once
#include "utils.hpp"
#include <cmath>

namespace rover {

// First-order IIR low-pass filter: y += alpha * (x - y), alpha = dt / (dt +
// 1/(2*pi*fc)).
class LowPass {
public:
  LowPass(double cutoffHz, double dt) {
    double rc = 1.0 / (2.0 * PI * cutoffHz);
    alpha_ = dt / (dt + rc);
  }
  double update(double x) {
    if (!init_) {
      y_ = x;
      init_ = true;
    } else {
      y_ += alpha_ * (x - y_);
    }
    return y_;
  }
  double value() const { return y_; }
  void reset() { init_ = false; }

private:
  double alpha_ = 1.0;
  double y_ = 0.0;
  bool init_ = false;
};

} // namespace rover
