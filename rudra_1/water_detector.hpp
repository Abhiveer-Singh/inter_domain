#pragma once
// Camera water detector. No OpenCV. Rover-only perception: colour + geometry.
#include <algorithm>
#include <cmath>
#include <vector>

struct WaterResult {
  bool waterSeen = false;
  double waterBearing = 0.0; // rad, +left of rover heading
  double waterRange = 0.0;   // m, level-ground distance camera -> nearest water
  double waterNear = 0.0;    // fraction of bottom-centre pixels that are water
  int waterPixels = 0;       // pixels in the kept region (full-res equivalent)
};

class WaterDetector {
public:
  static constexpr double CAM_HEIGHT = 0.25;
  static constexpr double CAM_TILT_UP = 0.15;
  static constexpr int STEP = 2;
  static constexpr int MIN_CELLS = 60;
  static constexpr int MIN_VALID = 20;
  static constexpr double MAX_RANGE = 15.0;

  static bool isWater(int r, int g, int b) {
    const int v = std::max(r, std::max(g, b));
    return v >= 140 && b - g >= 20 && b - g <= 45 && g - r >= 4 &&
           g - r <= 30 && b > r + 25;
  }

  WaterResult process(const unsigned char *bgra, int w, int h, double roll,
                      double pitch, double fovX) {
    WaterResult out;
    if (!bgra || w < 8 || h < 8) return out;
    const int cw = w / STEP, ch = h / STEP;
    mask_.assign(static_cast<size_t>(cw) * ch, 0);
    seen_.assign(static_cast<size_t>(cw) * ch, 0);

    int nearTot = 0, nearWet = 0;
    for (int cy = 0; cy < ch; ++cy)
      for (int cx = 0; cx < cw; ++cx) {
        const int x = cx * STEP, y = cy * STEP;
        const unsigned char *p = bgra + 4 * (static_cast<size_t>(y) * w + x);
        const bool wet = isWater(p[2], p[1], p[0]);
        mask_[static_cast<size_t>(cy) * cw + cx] = wet;
        if (y >= static_cast<int>(h * 0.85) && x >= w * 3 / 10 && x < w * 7 / 10) {
          ++nearTot;
          if (wet) ++nearWet;
        }
      }
    out.waterNear = nearTot ? static_cast<double>(nearWet) / nearTot : 0.0;

    best_.clear();
    for (int s = 0; s < cw * ch; ++s) {
      if (!mask_[s] || seen_[s]) continue;
      comp_.clear();
      stack_.clear();
      stack_.push_back(s);
      seen_[s] = 1;
      while (!stack_.empty()) {
        const int i = stack_.back();
        stack_.pop_back();
        comp_.push_back(i);
        const int x = i % cw, y = i / cw;
        const int nb[4] = {x > 0 ? i - 1 : -1, x < cw - 1 ? i + 1 : -1,
                           y > 0 ? i - cw : -1, y < ch - 1 ? i + cw : -1};
        for (int k = 0; k < 4; ++k) {
          const int j = nb[k];
          if (j >= 0 && mask_[j] && !seen_[j]) {
            seen_[j] = 1;
            stack_.push_back(j);
          }
        }
      }
      if (comp_.size() > best_.size()) best_.swap(comp_);
    }
    if (static_cast<int>(best_.size()) < MIN_CELLS) return out;

    const double fx = (w / 2.0) / std::tan(fovX / 2.0);
    const double ct = std::cos(CAM_TILT_UP), st = std::sin(CAM_TILT_UP);
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);

    double minRange = 1e9, sumB = 0.0;
    int nValid = 0;
    for (int i : best_) {
      const double px = (i % cw) * STEP, py = (i / cw) * STEP;
      const double l = -(px - w / 2.0) / fx;
      const double u = -(py - h / 2.0) / fx;
      const double bx = ct - st * u, by = l, bz = st + ct * u;
      const double rx = bx, ry = cr * by - sr * bz, rz = sr * by + cr * bz;
      const double lx = cp * rx + sp * rz, ly = ry, lz = -sp * rx + cp * rz;
      if (lz < -0.02) {
        const double rng = CAM_HEIGHT * std::hypot(lx, ly) / (-lz);
        minRange = std::min(minRange, rng);
        sumB += std::atan2(ly, lx);
        ++nValid;
      }
    }
    if (nValid < MIN_VALID) return out;
    out.waterSeen = true;
    out.waterRange = std::min(MAX_RANGE, minRange);
    out.waterBearing = sumB / nValid;
    out.waterPixels = static_cast<int>(best_.size()) * STEP * STEP;
    return out;
  }

private:
  std::vector<unsigned char> mask_, seen_;
  std::vector<int> stack_, comp_, best_;
};
