// move_ponds.cpp - move the water bodies of a Webots world closer to the rover.
//
// In your world each pond is a flat transparent Plane (Solid "water1"/"water2")
// at a fixed height, and the water only *shows* where the ElevationGrid terrain
// dips below that plane. So moving a pond means: (1) find its basin in the
// height array, (2) copy the basin to the new place, (3) smooth the old place
// back to ordinary ground, (4) shift the Plane.
//
// Build : g++ -std=c++17 -O2 -o move_ponds move_ponds.cpp
// Run   : ./move_ponds updated_rover_terrain_water.wbt
// updated_rover_terrain_water_close.wbt Pure C++, no dependencies. The input
// file is never modified.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ------------------------------------------------------------------ settings
// ----
struct Target {
  const char *name;
  double x, y;
};
// Where the CENTRE of each pond should end up (world metres). Terrain spans
// -25..25.
static const Target TARGETS[] = {{"water1", 0.0, -10.0}, {"water2", -8.0, 9.0}};
constexpr int WALL_CELLS =
    6; // width of the sloping bank copied with each basin
constexpr double LEVEL_TOL =
    0.02; // cells this close to the water plane count as "basin"
constexpr double FILL_MARGIN =
    0.10; // old basins are filled at least this far above the water
constexpr bool MOVE_ROVER =
    true; // also move the rover to flat, open ground (see step 3)
constexpr int SPAWN_RADIUS_CELLS =
    8; // 4 m: the area around the spawn that must be flat
constexpr double SPAWN_MIN_SHORE =
    8.0; // rover starts this far (m) from the nearest water ...
constexpr double SPAWN_MAX_SHORE = 14.0; // ... but no further than this

// ------------------------------------------------------------------- helpers
// ----
static std::string slurp(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot open " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static size_t findOrDie(const std::string &s, const std::string &key,
                        size_t from = 0) {
  size_t p = s.find(key, from);
  if (p == std::string::npos)
    throw std::runtime_error("not found in world file: " + key);
  return p;
}

// Reads `count` numbers following `key`; fills their text span [spanBeg,
// spanEnd).
static std::vector<double> numbersAt(const std::string &s, size_t keyPos,
                                     size_t keyLen, int count,
                                     size_t *spanBeg = nullptr,
                                     size_t *spanEnd = nullptr) {
  const char *base = s.c_str();
  const char *p = base + keyPos + keyLen;
  while (*p == ' ' || *p == '\t')
    ++p;
  if (spanBeg)
    *spanBeg = static_cast<size_t>(p - base);
  std::vector<double> v;
  for (int i = 0; i < count; ++i) {
    char *e;
    double d = std::strtod(p, &e);
    if (e == p)
      throw std::runtime_error("number expected after keyword");
    v.push_back(d);
    p = e;
  }
  if (spanEnd)
    *spanEnd = static_cast<size_t>(p - base);
  return v;
}

static std::string fmt(const char *f, double a) {
  char b[64];
  std::snprintf(b, sizeof b, f, a);
  return b;
}

struct Replace {
  size_t beg, end;
  std::string text;
};

struct Pond {
  std::string name;
  double px, py, pz, size; // water plane centre / height / edge length
  size_t trBeg, trEnd;     // text span of its "translation" numbers
  std::vector<int> core;   // basin cells (indices into the height array)
  int dx = 0, dy = 0;      // shift in grid cells
  std::vector<int> region; // core + bank
  std::vector<double>
      weight; // per region cell, 1 in core -> 0 at outer bank edge
};

int main(int argc, char **argv) {
  if (argc < 3) {
    std::printf("usage: %s <in.wbt> <out.wbt>\n", argv[0]);
    return 1;
  }
  try {
    const std::string in = slurp(argv[1]);

    // ---- terrain
    // ---------------------------------------------------------------
    size_t egPos = findOrDie(in, "ElevationGrid");
    size_t trPos = findOrDie(in, "translation");
    if (trPos > egPos)
      throw std::runtime_error(
          "terrain translation not found before ElevationGrid");
    std::vector<double> tr = numbersAt(in, trPos, 11, 3);
    const double tx = tr[0], ty = tr[1];
    const int nx = static_cast<int>(
        numbersAt(in, findOrDie(in, "xDimension", egPos), 10, 1)[0]);
    const int ny = static_cast<int>(
        numbersAt(in, findOrDie(in, "yDimension", egPos), 10, 1)[0]);
    const double xs = numbersAt(in, findOrDie(in, "xSpacing", egPos), 8, 1)[0];
    const double ys = numbersAt(in, findOrDie(in, "ySpacing", egPos), 8, 1)[0];

    size_t hKey = findOrDie(in, "height [", egPos);
    size_t hBeg = hKey + 8, hEnd = findOrDie(in, "]", hBeg);
    std::vector<double> orig;
    {
      const char *p = in.c_str() + hBeg;
      const char *end = in.c_str() + hEnd;
      while (p < end) {
        while (p < end &&
               (*p == ',' || std::isspace(static_cast<unsigned char>(*p))))
          ++p;
        if (p >= end)
          break;
        char *e;
        double d = std::strtod(p, &e);
        if (e == p)
          throw std::runtime_error("bad number in height array");
        orig.push_back(d);
        p = e;
      }
    }
    if (static_cast<int>(orig.size()) != nx * ny)
      throw std::runtime_error("height count " + std::to_string(orig.size()) +
                               " != " + std::to_string(nx) + "x" +
                               std::to_string(ny));
    auto X = [&](int ix) { return tx + ix * xs; };
    auto Y = [&](int iy) { return ty + iy * ys; };
    std::printf("terrain %dx%d cells, spacing %.2f m, spans x[%.1f,%.1f] "
                "y[%.1f,%.1f]\n",
                nx, ny, xs, X(0), X(nx - 1), Y(0), Y(ny - 1));

    // ---- rover start (for the distance report)
    // ---------------------------------
    double rvx = 0, rvy = 0;
    size_t rvTrBeg = 0, rvTrEnd = 0, rvRotBeg = 0, rvRotEnd = 0;
    bool haveRot = false;
    {
      size_t rb = findOrDie(in, "Robot {");
      size_t rt = findOrDie(in, "translation", rb);
      std::vector<double> r = numbersAt(in, rt, 11, 3, &rvTrBeg, &rvTrEnd);
      size_t rr = in.find("rotation", rb), rch = in.find("children", rb);
      if (rr != std::string::npos &&
          rr < rch) { // the Robot's own rotation (before its children)
        numbersAt(in, rr, 8, 4, &rvRotBeg, &rvRotEnd);
        haveRot = true;
      }
      rvx = r[0];
      rvy = r[1];
      std::printf("rover starts at (%.2f, %.2f)\n", rvx, rvy);
    }

    // ---- ponds
    // -------------------------------------------------------------------
    std::vector<Pond> ponds;
    for (const Target &t : TARGETS) {
      Pond p;
      p.name = t.name;
      size_t np = findOrDie(in, std::string("name \"") + t.name + "\"");
      size_t tp = in.rfind("translation", np);
      size_t sp = in.rfind("size", np);
      if (tp == std::string::npos || sp == std::string::npos)
        throw std::runtime_error("water node layout unexpected");
      std::vector<double> v = numbersAt(in, tp, 11, 3, &p.trBeg, &p.trEnd);
      p.px = v[0];
      p.py = v[1];
      p.pz = v[2];
      p.size = numbersAt(in, sp, 4, 1)[0];

      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix)
          if (std::fabs(X(ix) - p.px) <= p.size / 2 &&
              std::fabs(Y(iy) - p.py) <= p.size / 2 &&
              orig[iy * nx + ix] <= p.pz + LEVEL_TOL)
            p.core.push_back(iy * nx + ix);
      if (p.core.empty())
        throw std::runtime_error(
            p.name + ": no terrain below its water plane - nothing to move");

      double cx = 0, cy = 0;
      for (int c : p.core) {
        cx += c % nx;
        cy += c / nx;
      }
      cx /= p.core.size();
      cy /= p.core.size();
      p.dx = static_cast<int>(std::lround((t.x - X(0)) / xs - cx));
      p.dy = static_cast<int>(std::lround((t.y - Y(0)) / ys - cy));

      // core + bank, with a smooth 1 -> 0 weight
      int x0 = nx, x1 = -1, y0 = ny, y1 = -1;
      for (int c : p.core) {
        x0 = std::min(x0, c % nx);
        x1 = std::max(x1, c % nx);
        y0 = std::min(y0, c / nx);
        y1 = std::max(y1, c / nx);
      }
      for (int iy = std::max(0, y0 - WALL_CELLS);
           iy <= std::min(ny - 1, y1 + WALL_CELLS); ++iy)
        for (int ix = std::max(0, x0 - WALL_CELLS);
             ix <= std::min(nx - 1, x1 + WALL_CELLS); ++ix) {
          double d = 1e9;
          for (int c : p.core)
            d = std::min(d, std::hypot(ix - c % nx, iy - c / nx));
          if (d > WALL_CELLS)
            continue;
          double u = 1.0 - d / WALL_CELLS;
          p.region.push_back(iy * nx + ix);
          p.weight.push_back(u * u * (3 - 2 * u)); // smoothstep
        }
      std::printf("%s: %zu basin cells (%.0f m^2), centre (%.1f, %.1f) -> "
                  "(%.1f, %.1f)\n",
                  p.name.c_str(), p.core.size(), p.core.size() * xs * ys,
                  X(0) + cx * xs, Y(0) + cy * ys, t.x, t.y);
      ponds.push_back(p);
    }

    // ---- 1) smooth the OLD basins back to ground (Laplace relaxation)
    // ---------------
    std::vector<double> cur = orig;
    for (const Pond &p : ponds) {
      std::vector<char> inReg(static_cast<size_t>(nx) * ny, 0);
      for (int c : p.region)
        inReg[c] = 1;
      double sum = 0;
      int cnt = 0;
      for (int c : p.region) {
        int ix = c % nx, iy = c / nx;
        const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (auto &o : nb) {
          int jx = ix + o[0], jy = iy + o[1];
          if (jx >= 0 && jy >= 0 && jx < nx && jy < ny &&
              !inReg[jy * nx + jx]) {
            sum += orig[jy * nx + jx];
            ++cnt;
          }
        }
      }
      double mean = cnt ? sum / cnt : p.pz + 0.4;
      for (int c : p.region)
        cur[c] = mean;
      for (int it = 0; it < 3000; ++it)
        for (int c : p.region) {
          int ix = c % nx, iy = c / nx;
          const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
          double s = 0;
          int n = 0;
          for (auto &o : nb) {
            int jx = ix + o[0], jy = iy + o[1];
            if (jx >= 0 && jy >= 0 && jx < nx && jy < ny) {
              s += cur[jy * nx + jx];
              ++n;
            }
          }
          if (n)
            cur[c] = s / n;
        }
      for (int c : p.region)
        cur[c] = std::max(cur[c], p.pz + FILL_MARGIN);
    }

    // ---- 2) carve the basins at the NEW places
    // ----------------------------------------
    for (const Pond &p : ponds)
      for (size_t k = 0; k < p.region.size(); ++k) {
        int ix = p.region[k] % nx + p.dx, iy = p.region[k] / nx + p.dy;
        if (ix < 0 || iy < 0 || ix >= nx || iy >= ny)
          continue; // part of the bank falls off the map
        double w = p.weight[k];
        cur[iy * nx + ix] =
            w * orig[p.region[k]] + (1.0 - w) * cur[iy * nx + ix];
      }

    // ---- 3) put the rover on flat, open ground near the ponds
    // ------------------------------ The rover's 2D lidar reads any slope
    // steeper than ~10 deg as a solid wall, so a start inside a hollow boxes
    // the planner in. Pick the flattest spot with open ground all round.
    std::string roverTr, roverRot;
    if (MOVE_ROVER) {
      std::vector<std::pair<int, int>> water; // new water cells
      double maxLevel = -1e9;
      for (const Pond &p : ponds) {
        maxLevel = std::max(maxLevel, p.pz);
        for (int c : p.core) {
          int ix = c % nx + p.dx, iy = c / nx + p.dy;
          if (ix >= 0 && iy >= 0 && ix < nx && iy < ny)
            water.push_back({ix, iy});
        }
      }
      std::vector<double> slope(static_cast<size_t>(nx) * ny, 0.0); // radians
      for (int iy = 1; iy < ny - 1; ++iy)
        for (int ix = 1; ix < nx - 1; ++ix) {
          double gx =
              (cur[iy * nx + ix + 1] - cur[iy * nx + ix - 1]) / (2 * xs);
          double gy =
              (cur[(iy + 1) * nx + ix] - cur[(iy - 1) * nx + ix]) / (2 * ys);
          slope[iy * nx + ix] = std::atan(std::hypot(gx, gy));
        }
      const int R = SPAWN_RADIUS_CELLS;
      int bx = -1, by = -1;
      double bestScore = 1e18, bestRelief = 0, bestSlope = 0, bestD = 0;
      for (int pass = 0; pass < 2 && bx < 0; ++pass) {
        const double dMin = pass == 0 ? SPAWN_MIN_SHORE : 5.0,
                     dMax = pass == 0 ? SPAWN_MAX_SHORE : 25.0;
        for (int iy = R + 4; iy < ny - R - 4; ++iy)
          for (int ix = R + 4; ix < nx - R - 4; ++ix) {
            double hmin = 1e9, hmax = -1e9, smax = 0;
            for (int dy = -R; dy <= R; ++dy)
              for (int dx = -R; dx <= R; ++dx) {
                if (std::hypot(dx, dy) > R)
                  continue;
                double h = cur[(iy + dy) * nx + ix + dx];
                hmin = std::min(hmin, h);
                hmax = std::max(hmax, h);
                smax = std::max(smax, slope[(iy + dy) * nx + ix + dx]);
              }
            if (hmin < maxLevel + 0.3)
              continue; // basin or water inside the window
            double d = 1e9;
            for (const auto &w : water)
              d = std::min(
                  d, std::hypot((ix - w.first) * xs, (iy - w.second) * ys));
            if (d < dMin || d > dMax)
              continue;
            double score = (hmax - hmin) + 2.0 * smax;
            if (score < bestScore) {
              bestScore = score;
              bx = ix;
              by = iy;
              bestRelief = hmax - hmin;
              bestSlope = smax;
              bestD = d;
            }
          }
      }
      if (bx < 0) {
        std::printf("WARNING: no flat open spot found near the ponds - rover "
                    "left where it was\n");
      } else {
        double wx = 0, wy = 0, wd = 1e9; // nearest water cell, to face it
        for (const auto &w : water) {
          double d = std::hypot((bx - w.first) * xs, (by - w.second) * ys);
          if (d < wd) {
            wd = d;
            wx = X(w.first);
            wy = Y(w.second);
          }
        }
        double nxp = X(bx), nyp = Y(by), nzp = cur[by * nx + bx] + 0.06;
        double yaw = std::atan2(wy - nyp, wx - nxp);
        roverTr =
            fmt("%.4f", nxp) + " " + fmt("%.4f", nyp) + " " + fmt("%.4f", nzp);
        roverRot = "0 0 1 " + fmt("%.4f", yaw);
        rvx = nxp;
        rvy = nyp;
        std::printf("rover moved to (%.2f, %.2f, z %.2f): relief %.2f m and "
                    "max slope %.1f deg within %.1f m, "
                    "%.1f m from the nearest water, facing it\n",
                    nxp, nyp, nzp, bestRelief, bestSlope * 180.0 / 3.14159265,
                    R * xs, bestD);
      }
    }

    // ---- 4) write the new file
    // -------------------------------------------------------------
    std::vector<Replace> reps;
    if (!roverTr.empty()) {
      reps.push_back({rvTrBeg, rvTrEnd, roverTr});
      if (haveRot)
        reps.push_back({rvRotBeg, rvRotEnd, roverRot});
    }
    {
      std::string h = "\n          ";
      for (size_t i = 0; i < cur.size(); ++i) {
        h += fmt("%.3f", cur[i]);
        if (i + 1 < cur.size())
          h += ", ";
      }
      h += "\n        ";
      reps.push_back({hBeg, hEnd, h});
    }
    for (const Pond &p : ponds) {
      reps.push_back({p.trBeg, p.trEnd,
                      fmt("%.4f", p.px + p.dx * xs) + " " +
                          fmt("%.4f", p.py + p.dy * ys) + " " +
                          fmt("%.4f", p.pz)});
    }
    std::sort(reps.begin(), reps.end(),
              [](const Replace &a, const Replace &b) { return a.beg > b.beg; });
    std::string out = in;
    for (const Replace &r : reps)
      out.replace(r.beg, r.end - r.beg, r.text);
    std::ofstream(argv[2], std::ios::binary) << out;

    // ---- report
    // ---------------------------------------------------------------------------
    for (const Pond &p : ponds) {
      double best = 1e9;
      int n = 0;
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix) {
          double px = p.px + p.dx * xs, py = p.py + p.dy * ys;
          if (std::fabs(X(ix) - px) <= p.size / 2 &&
              std::fabs(Y(iy) - py) <= p.size / 2 &&
              cur[iy * nx + ix] <= p.pz + LEVEL_TOL) {
            ++n;
            best = std::min(best, std::hypot(X(ix) - rvx, Y(iy) - rvy));
          }
        }
      std::printf("%s now: %d water cells, nearest shore %.1f m from the rover "
                  "(plane at %.2f, %.2f)\n",
                  p.name.c_str(), n, best, p.px + p.dx * xs, p.py + p.dy * ys);
    }
    std::printf("wrote %s\n", argv[2]);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
