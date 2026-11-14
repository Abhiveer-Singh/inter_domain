#pragma once
#include "occupancy_grid.hpp"
#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

namespace rover {

struct PlanResult {
  bool found = false;
  std::vector<Vec2> path; // world coordinates, start -> goal
};

// 8-connected A* on the cost map from OccupancyGrid::buildCostMap().
// - octile heuristic (admissible: penalties only add cost)
// - no diagonal corner-cutting
// - soft penalty near obstacles keeps the path away from walls
// - line-of-sight smoothing afterwards
class AStar {
public:
  explicit AStar(double penaltyWeight = 1.5) : penaltyWeight_(penaltyWeight) {}

  PlanResult plan(const OccupancyGrid &g, std::vector<float> cost,
                  const Vec2 &start, const Vec2 &goal,
                  int clearStartCells = 5) const {
    PlanResult res;
    const int n = g.size();
    int sx, sy, gx, gy;
    if (!g.worldToCell(start, sx, sy) || !g.worldToCell(goal, gx, gy))
      return res;

    // The rover may sit inside an inflated zone; let it get out.
    for (int dy = -clearStartCells; dy <= clearStartCells; ++dy)
      for (int dx = -clearStartCells; dx <= clearStartCells; ++dx) {
        int ix = sx + dx, iy = sy + dy;
        if (g.in(ix, iy) && cost[g.idx(ix, iy)] < 0.0f &&
            std::hypot(dx, dy) <= clearStartCells)
          cost[g.idx(ix, iy)] = 0.0f;
      }

    // If the goal cell is blocked, move it to the nearest free cell.
    if (cost[g.idx(gx, gy)] < 0.0f) {
      bool ok = false;
      for (int r = 1; r <= 30 && !ok; ++r)
        for (int dy = -r; dy <= r && !ok; ++dy)
          for (int dx = -r; dx <= r && !ok; ++dx) {
            if (std::max(std::abs(dx), std::abs(dy)) != r)
              continue;
            int ix = gx + dx, iy = gy + dy;
            if (g.in(ix, iy) && cost[g.idx(ix, iy)] >= 0.0f) {
              gx = ix;
              gy = iy;
              ok = true;
            }
          }
      if (!ok)
        return res;
    }

    const int N = n * n;
    const int startIdx = g.idx(sx, sy), goalIdx = g.idx(gx, gy);
    const double INF = 1e18, SQ2 = std::sqrt(2.0), res_m = g.res();
    std::vector<double> gScore(N, INF);
    std::vector<int> parent(N, -1);
    std::vector<char> closed(N, 0);

    auto h = [&](int ix, int iy) {
      double dx = std::abs(ix - gx), dy = std::abs(iy - gy);
      return res_m * ((dx + dy) + (SQ2 - 2.0) * std::min(dx, dy));
    };

    using QN = std::pair<double, int>;
    std::priority_queue<QN, std::vector<QN>, std::greater<QN>> open;
    gScore[startIdx] = 0.0;
    open.push({h(sx, sy), startIdx});

    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool reached = false;
    while (!open.empty()) {
      int cur = open.top().second;
      open.pop();
      if (closed[cur])
        continue;
      closed[cur] = 1;
      if (cur == goalIdx) {
        reached = true;
        break;
      }
      int cx = cur % n, cy = cur / n;
      for (int k = 0; k < 8; ++k) {
        int nx = cx + DX[k], ny = cy + DY[k];
        if (!g.in(nx, ny))
          continue;
        int ni = g.idx(nx, ny);
        if (closed[ni] || cost[ni] < 0.0f)
          continue;
        if (k >= 4 && (cost[g.idx(cx + DX[k], cy)] < 0.0f ||
                       cost[g.idx(cx, cy + DY[k])] < 0.0f))
          continue; // no corner cutting
        double step =
            (k < 4 ? 1.0 : SQ2) * res_m * (1.0 + penaltyWeight_ * cost[ni]);
        double t = gScore[cur] + step;
        if (t < gScore[ni]) {
          gScore[ni] = t;
          parent[ni] = cur;
          open.push({t + h(nx, ny), ni});
        }
      }
    }
    if (!reached)
      return res;

    std::vector<int> cells;
    for (int c = goalIdx; c != -1; c = parent[c])
      cells.push_back(c);
    std::reverse(cells.begin(), cells.end());

    std::vector<Vec2> raw;
    raw.reserve(cells.size());
    for (int c : cells)
      raw.push_back(g.cellToWorld(c % n, c / n));
    res.path = smooth(g, cost, raw);
    res.found = true;
    return res;
  }

private:
  // Greedy line-of-sight pruning: keep only the waypoints that are needed.
  static std::vector<Vec2> smooth(const OccupancyGrid &g,
                                  const std::vector<float> &cost,
                                  const std::vector<Vec2> &p) {
    if (p.size() <= 2)
      return p;
    std::vector<Vec2> out{p.front()};
    size_t i = 0;
    while (i < p.size() - 1) {
      size_t j = p.size() - 1;
      while (j > i + 1 && !lineFree(g, cost, p[i], p[j]))
        --j;
      out.push_back(p[j]);
      i = j;
    }
    return out;
  }

  static bool lineFree(const OccupancyGrid &g, const std::vector<float> &cost,
                       const Vec2 &a, const Vec2 &b) {
    double d = dist(a, b);
    int steps = std::max(1, static_cast<int>(d / (g.res() * 0.5)));
    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;
      Vec2 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
      int ix, iy;
      if (!g.worldToCell(p, ix, iy))
        return false;
      if (cost[g.idx(ix, iy)] < 0.0f)
        return false;
    }
    return true;
  }

  double penaltyWeight_;
};

} // namespace rover
