#pragma once
#include <algorithm>
#include <cmath>

namespace rover {

constexpr double PI = 3.14159265358979323846;

inline double wrapAngle(double a) {
  while (a > PI)
    a -= 2.0 * PI;
  while (a < -PI)
    a += 2.0 * PI;
  return a;
}

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

inline double dist(const Vec2 &a, const Vec2 &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

} // namespace rover
