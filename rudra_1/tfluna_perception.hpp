#pragma once
// TF-Luna perception for a mechanically scanned pan/tilt sensor.
// Pure C++: the Webots controller (or hardware driver) supplies each range
// reading together with the pan/tilt angles at which it was measured.

#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace rover {

// Coordinate convention used here:
//   rover X = forward, rover Y = left, rover Z = up
//   panAngle > 0 = sensor points left
//   tiltAngle > 0 = sensor points up
//
// The angles must be in radians. Convert servo degrees to radians before
// calling addMeasurement().

struct TfLunaMeasurement {
  double range = std::numeric_limits<double>::quiet_NaN(); // metres
  double panAngle = 0.0;                                   // radians, +left
  double tiltAngle = 0.0;                                  // radians, +up
  double timestamp = 0.0; // seconds; optional but useful for logging
};

struct PointXYZ {
  double x = 0.0; // forward
  double y = 0.0; // left
  double z = 0.0; // up
};

struct ScanSummary {
  double front = 99.0;    // nearest valid return in the forward corridor [m]
  double left = 99.0;     // nearest valid return in the left sector [m]
  double right = 99.0;    // nearest valid return in the right sector [m]
  double bestAngle = 0.0; // most open sampled horizontal direction [rad], +left
  bool valid = false;
  std::size_t validMeasurements = 0;
};

class TfLunaScanner {
public:
  explicit TfLunaScanner(double minRange = 0.20, double maxRange = 8.0,
                         double corridorHalfWidth = 0.35)
      : minRange_(minRange), maxRange_(maxRange),
        corridorHalfWidth_(corridorHalfWidth) {}

  // Start a fresh mechanical sweep. Call once before moving the pan/tilt
  // servos through the scan pattern.
  void beginScan() {
    measurements_.clear();
    points_.clear();
  }

  // Add one TF-Luna reading. panAngle and tiltAngle are the measured or
  // commanded servo angles in radians. Invalid/out-of-range returns are
  // ignored.
  bool addMeasurement(double range, double panAngle, double tiltAngle,
                      double timestamp = 0.0) {
    if (!std::isfinite(range) || range < minRange_ || range > maxRange_ ||
        !std::isfinite(panAngle) || !std::isfinite(tiltAngle)) {
      return false;
    }

    TfLunaMeasurement m;
    m.range = range;
    m.panAngle = panAngle;
    m.tiltAngle = tiltAngle;
    m.timestamp = timestamp;
    measurements_.push_back(m);

    const double horizontal = range * std::cos(tiltAngle);
    PointXYZ p;
    p.x = horizontal * std::cos(panAngle);
    p.y = horizontal * std::sin(panAngle);
    p.z = range * std::sin(tiltAngle);
    points_.push_back(p);
    return true;
  }

  // Summarise a completed scan. The front/left/right values are based on
  // horizontal bearing (pan angle); tilt is retained in the 3D points.
  ScanSummary summarize(double frontHalfAngle = 0.45,
                        double sideStartAngle = 0.45,
                        double sideEndAngle = 1.75) const {
    ScanSummary out;
    if (measurements_.empty())
      return out;

    constexpr int BINS = 36; // 10-degree bins over the full circle
    std::vector<double> nearest(BINS, maxRange_);
    std::vector<char> seen(BINS, 0);

    for (const auto &m : measurements_) {
      const double a = wrapAngle(m.panAngle);
      const double horizontal = m.range * std::cos(m.tiltAngle);
      const double forward = horizontal * std::cos(a);
      const double lateral = horizontal * std::sin(a);

      // Only low/near-horizontal returns are useful for this 2D-style
      // obstacle summary. The full 3D points remain available separately.
      if (std::fabs(m.tiltAngle) <= 0.35) {
        if (forward > 0.0 && std::fabs(lateral) <= corridorHalfWidth_)
          out.front = std::min(out.front, horizontal);

        if (a >= sideStartAngle && a <= sideEndAngle)
          out.left = std::min(out.left, horizontal);

        if (a <= -sideStartAngle && a >= -sideEndAngle)
          out.right = std::min(out.right, horizontal);
      }

      int bin = static_cast<int>((a + PI) / (2.0 * PI) * BINS);
      bin = std::max(0, std::min(BINS - 1, bin));
      nearest[bin] = std::min(nearest[bin], horizontal);
      seen[bin] = 1;
    }

    double bestScore = -std::numeric_limits<double>::infinity();
    for (int b = 0; b < BINS; ++b) {
      if (!seen[b])
        continue;
      const double angle = (b + 0.5) / BINS * 2.0 * PI - PI;
      // Keep the local navigation choice in the forward hemisphere.
      if (std::fabs(angle) > 1.55)
        continue;
      const double score = nearest[b] - 0.8 * std::fabs(angle);
      if (score > bestScore) {
        bestScore = score;
        out.bestAngle = angle;
      }
    }

    out.validMeasurements = measurements_.size();
    out.valid = out.validMeasurements > 0;
    return out;
  }

  const std::vector<TfLunaMeasurement> &measurements() const {
    return measurements_;
  }

  // Points are expressed relative to the TF-Luna sensor origin. Add the
  // sensor's mounting translation/rotation in the controller before mapping
  // them into the rover/world frame.
  const std::vector<PointXYZ> &points() const { return points_; }

  std::size_t size() const { return measurements_.size(); }

private:
  double minRange_;
  double maxRange_;
  double corridorHalfWidth_;
  std::vector<TfLunaMeasurement> measurements_;
  std::vector<PointXYZ> points_;
};

} // namespace rover
