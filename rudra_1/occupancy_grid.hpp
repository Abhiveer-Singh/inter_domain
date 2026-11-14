#pragma once
#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace rover {

// Square log-odds occupancy grid centred on the world origin, plus:
//  - a static layer (known water bodies, never cleared by lidar)
//  - temporary "virtual" obstacles (e.g. camera-only detections), which expire.
class OccupancyGrid {
public:
  OccupancyGrid(double sizeM, double resolution)
      : res_(resolution), n_(static_cast<int>(sizeM / resolution)),
        half_(sizeM / 2.0), logOdds_(static_cast<size_t>(n_) * n_, 0.0f),
        staticBlocked_(static_cast<size_t>(n_) * n_, 0) {}

  int size() const { return n_; }
  double res() const { return res_; }
  bool in(int ix, int iy) const {
    return ix >= 0 && iy >= 0 && ix < n_ && iy < n_;
  }
  int idx(int ix, int iy) const { return iy * n_ + ix; }

  bool worldToCell(const Vec2 &p, int &ix, int &iy) const {
    ix = static_cast<int>(std::floor((p.x + half_) / res_));
    iy = static_cast<int>(std::floor((p.y + half_) / res_));
    return in(ix, iy);
  }
  Vec2 cellToWorld(int ix, int iy) const {
    return {ix * res_ - half_ + res_ / 2.0, iy * res_ - half_ + res_ / 2.0};
  }

  // Lidar ray: cells along the ray are free; the end cell is occupied if `hit`.
  void integrateRay(const Vec2 &from, const Vec2 &to, bool hit) {
    int x0, y0, x1, y1;
    // Clip the end point to the map instead of dropping the whole ray.
    if (!worldToCell(from, x0, y0))
      return;
    bool endInside = worldToCell(to, x1, y1);
    if (!endInside) {
      x1 = clampI(x1);
      y1 = clampI(y1);
      hit = false;
    }
    int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
      bool last = (x0 == x1 && y0 == y1);
      float &l = logOdds_[idx(x0, y0)];
      if (last && hit)
        l = std::min(L_MAX, l + L_OCC);
      else
        l = std::max(L_MIN, l + L_FREE);
      if (last)
        break;
      int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  void addStaticDisk(const Vec2 &c, double r) {
    forDisk(c, r, [&](int ix, int iy) { staticBlocked_[idx(ix, iy)] = 1; });
  }

  void addVirtualObstacle(const Vec2 &p, double r, double expiryTime) {
    virtual_.push_back({p, r, expiryTime});
  }

  // Result: <0 blocked, >=0 traversal penalty (0 = free, larger = nearer to
  // obstacles).
  std::vector<float> buildCostMap(double now, double robotRadius,
                                  double inflationRadius, double maxPenalty) {
    virtual_.erase(
        std::remove_if(virtual_.begin(), virtual_.end(),
                       [&](const Virt &v) { return v.expiry < now; }),
        virtual_.end());

    std::vector<int> seeds;
    for (int i = 0; i < n_ * n_; ++i)
      if (logOdds_[i] > OCC_THRESH || staticBlocked_[i])
        seeds.push_back(i);
    for (const auto &v : virtual_)
      forDisk(v.p, v.r, [&](int ix, int iy) { seeds.push_back(idx(ix, iy)); });

    std::vector<float> cost(static_cast<size_t>(n_) * n_, 0.0f);
    int R = static_cast<int>(std::ceil(inflationRadius / res_));
    for (int s : seeds) {
      int sx = s % n_, sy = s / n_;
      for (int dy = -R; dy <= R; ++dy)
        for (int dx = -R; dx <= R; ++dx) {
          int ix = sx + dx, iy = sy + dy;
          if (!in(ix, iy))
            continue;
          double d = std::hypot(dx, dy) * res_;
          if (d > inflationRadius)
            continue;
          float &c = cost[idx(ix, iy)];
          if (c < 0.0f)
            continue;
          if (d <= robotRadius) {
            c = -1.0f;
          } else {
            double pen = maxPenalty * (inflationRadius - d) /
                         (inflationRadius - robotRadius);
            c = std::max(c, static_cast<float>(pen));
          }
        }
    }
    return cost;
  }

  // Nearest frontier (free cell touching unknown space), reachable through free
  // cells, at least minDist away, inside +-arenaHalf, away from blacklisted points.
  bool nearestFrontier(const Vec2 &from, double arenaHalf, double minDist,
                       const std::vector<Vec2> &blacklist, Vec2 &out) const {
    int sx, sy;
    if (!worldToCell(from, sx, sy))
      return false;
    std::vector<int> steps(static_cast<size_t>(n_) * n_, -1);
    std::vector<int> q;
    q.push_back(idx(sx, sy));
    steps[idx(sx, sy)] = 0;
    const int minSteps = static_cast<int>(minDist / res_);
    for (size_t head = 0; head < q.size(); ++head) {
      int c = q[head], cx = c % n_, cy = c / n_;
      if (steps[c] >= minSteps && isFreeCell(cx, cy) && touchesUnknown(cx, cy) &&
          !nearOccupied(cx, cy, 3)) {
        Vec2 w = cellToWorld(cx, cy);
        bool ok = std::fabs(w.x) < arenaHalf && std::fabs(w.y) < arenaHalf;
        for (const auto &b : blacklist)
          if (std::hypot(b.x - w.x, b.y - w.y) < 1.5)
            ok = false;
        if (ok) {
          out = w;
          return true;
        }
      }
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          int nx = cx + dx, ny = cy + dy;
          if (!in(nx, ny) || steps[idx(nx, ny)] >= 0 || !isFreeCell(nx, ny) ||
              staticBlocked_[idx(nx, ny)])
            continue;
          steps[idx(nx, ny)] = steps[c] + 1;
          q.push_back(idx(nx, ny));
        }
    }
    return false;
  }

private:
  bool isFreeCell(int ix, int iy) const { return logOdds_[idx(ix, iy)] < -0.35f; }
  bool touchesUnknown(int ix, int iy) const {
    static const int d[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto &k : d) {
      int nx = ix + k[0], ny = iy + k[1];
      if (in(nx, ny) && std::fabs(logOdds_[idx(nx, ny)]) < 1e-6f)
        return true;
    }
    return false;
  }
  bool nearOccupied(int ix, int iy, int r) const {
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx) {
        int nx = ix + dx, ny = iy + dy;
        if (in(nx, ny) && (logOdds_[idx(nx, ny)] > OCC_THRESH || staticBlocked_[idx(nx, ny)]))
          return true;
      }
    return false;
  }
  struct Virt {
    Vec2 p;
    double r;
    double expiry;
  };

  template <typename F> void forDisk(const Vec2 &c, double r, F f) const {
    int cx, cy;
    cx = static_cast<int>(std::floor((c.x + half_) / res_));
    cy = static_cast<int>(std::floor((c.y + half_) / res_));
    int R = static_cast<int>(std::ceil(r / res_));
    for (int dy = -R; dy <= R; ++dy)
      for (int dx = -R; dx <= R; ++dx) {
        if (std::hypot(dx, dy) * res_ > r)
          continue;
        int ix = cx + dx, iy = cy + dy;
        if (in(ix, iy))
          f(ix, iy);
      }
  }
  int clampI(int v) const { return std::max(0, std::min(n_ - 1, v)); }

  static constexpr float L_OCC = 0.85f, L_FREE = -0.40f, L_MIN = -4.0f,
                         L_MAX = 4.0f;
  static constexpr float OCC_THRESH = 1.0f;

  double res_;
  int n_;
  double half_;
  std::vector<float> logOdds_;
  std::vector<char> staticBlocked_;
  std::vector<Virt> virtual_;
};

} // namespace rover
