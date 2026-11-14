#pragma once
#include "utils.hpp"
#include <algorithm>
#include <cmath>

namespace rover {

struct WheelSpeeds {
  double leftRadS = 0.0;  // angular velocity for every left wheel  [rad/s]
  double rightRadS = 0.0; // angular velocity for every right wheel [rad/s]
  double leftLin = 0.0;   // equivalent linear speed [m/s] (for logging)
  double rightLin = 0.0;
};

// Differential / skid-steer mixing.
//   v : forward speed [m/s]
//   w : yaw rate [rad/s], positive = turn LEFT (counter-clockwise)
// Turning left => left wheels slower, right wheels faster:
//   vL = v - w * T/2        vR = v + w * T/2
// `effectiveTrack` should be the geometric track width times a slip factor (>1)
// for skid steering.
inline WheelSpeeds mixSkidSteer(double v, double w, double effectiveTrack,
                                double wheelRadius, double maxWheelRadS) {
  double vL = v - w * effectiveTrack / 2.0;
  double vR = v + w * effectiveTrack / 2.0;

  double maxLin = maxWheelRadS * wheelRadius;
  double peak = std::max(std::fabs(vL), std::fabs(vR));
  if (peak > maxLin) { // scale both sides so the turn ratio is preserved
    double s = maxLin / peak;
    vL *= s;
    vR *= s;
  }
  WheelSpeeds out;
  out.leftLin = vL;
  out.rightLin = vR;
  out.leftRadS = vL / wheelRadius;
  out.rightRadS = vR / wheelRadius;
  return out;
}

} // namespace rover
