#pragma once
// Lidar scan summary + simple camera-based obstacle / water detector.
// Pure C++, no Webots types: the controller passes raw arrays in.
#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace rover {

// ---------------------------------------------------------------- lidar ----
struct ScanSummary {
  double front = 99.0; // closest return inside the rover's swept corridor [m]
  double left = 99.0;  // closest return on the left side
  double right = 99.0; // closest return on the right side
  double bestAngle =
      0.0; // robot-frame angle (rad, +left) of the most open direction
  bool valid = false;
};

// ranges: one horizontal layer, `n` rays spanning `fov`.
// firstRayIsLeft: Webots lidar ray 0 is the left-most one in the usual setup;
// flip if the map comes out mirrored.
inline double lidarRayAngle(int i, int n, double fov, bool firstRayIsLeft) {
  double step = fov / n;
  return firstRayIsLeft ? (fov / 2.0 - (i + 0.5) * step)
                        : (-fov / 2.0 + (i + 0.5) * step);
}

inline ScanSummary summarizeScan(const float *ranges, int n, double fov,
                                 double minRange, double maxRange,
                                 bool firstRayIsLeft,
                                 double corridorHalfWidth) {
  ScanSummary s;
  if (!ranges || n <= 0)
    return s;
  const int BINS =
      24; // 15 deg bins over +-180 deg (only the covered ones are filled)
  std::vector<double> bin(BINS, maxRange);
  std::vector<char> binSeen(BINS, 0);

  for (int i = 0; i < n; ++i) {
    double a = lidarRayAngle(i, n, fov, firstRayIsLeft);
    double r = ranges[i];
    if (!std::isfinite(r) || r > maxRange)
      r = maxRange;
    if (r < minRange)
      continue;

    double lateral = r * std::sin(a), forward = r * std::cos(a);
    if (forward > 0.0 && std::fabs(lateral) < corridorHalfWidth)
      s.front = std::min(s.front, r);
    if (a > 0.4 && a < 1.8)
      s.left = std::min(s.left, r);
    if (a < -0.4 && a > -1.8)
      s.right = std::min(s.right, r);

    int b = static_cast<int>((wrapAngle(a) + PI) / (2.0 * PI) * BINS);
    b = std::max(0, std::min(BINS - 1, b));
    bin[b] = std::min(bin[b], r);
    binSeen[b] = 1;
  }

  double bestScore = -1e9;
  for (int b = 0; b < BINS; ++b) {
    if (!binSeen[b])
      continue;
    double a = (b + 0.5) / BINS * 2.0 * PI - PI;
    if (std::fabs(a) > 2.0)
      continue;
    double score =
        bin[b] - 0.8 * std::fabs(a); // prefer open AND roughly forward
    if (score > bestScore) {
      bestScore = score;
      s.bestAngle = a;
    }
  }
  s.valid = true;
  return s;
}

// --------------------------------------------------------------- camera ----
struct CameraResult {
  double obstacle[3] = {
      0, 0, 0}; // fraction of "not ground" pixels: left, centre, right
  double waterNear =
      0.0; // fraction of water pixels at the bottom-centre (water within ~2 m)
  bool waterSeen = false; // enough water-coloured pixels below the sky boundary
  double waterBearing = 0.0; // rad, +left of the rover heading
  double waterRange = 0.0;   // m, ground distance camera -> nearest water pixel
                             // (flat-ground model)
  int waterPixels = 0;       // DEBUG: number of sampled water pixels found
  bool valid = false;
};

class CameraDetector {
public:
  // bgra: Webots camera image. roverPitch: IMU pitch (rad, + = nose down).
  CameraResult process(const unsigned char *bgra, int w, int h,
                       double roverPitch = 0.0, double fovX = 1.0) {
    CameraResult out;
    if (!bgra || w < 8 || h < 8)
      return out;

    auto px = [&](int x, int y, int &r, int &g, int &b) {
      const unsigned char *p = bgra + 4 * (static_cast<size_t>(y) * w + x);
      b = p[0];
      g = p[1];
      r = p[2];
    };

    // 0) water: search below the per-column sky boundary (hills can put the
    //    lake above the flat-ground horizon, so no horizon cutoff here)
    {
      const double fx =
          (w / 2.0) / std::tan(fovX / 2.0); // square pixels: fy == fx
      const double horizon = h / 2.0 + fx * std::tan(CAM_TILT_UP - roverPitch);
      std::vector<int> rowCnt(h, 0);
      long sumX = 0;
      int total = 0, nearTot = 0, nearWet = 0;

      for (int x = 0; x < w; x += 2) {
        int y = 0;
        int r, g, b;
        while (y < h) { // skip the sky at the top of this column
          px(x, y, r, g, b);
          if (!isSky(r, g, b))
            break;
          ++y;
        }
        for (y += 3; y < h; y += 2) {
          px(x, y, r, g, b);
          bool wet = isWater(r, g, b);
          if (y >= static_cast<int>(h * 0.85) && x >= w * 3 / 10 &&
              x < w * 7 / 10) {
            ++nearTot;
            if (wet)
              ++nearWet;
          }
          if (wet) {
            ++total;
            sumX += x;
            ++rowCnt[y];
          }
        }
      }
      out.waterPixels = total;
      out.waterNear = nearTot ? static_cast<double>(nearWet) / nearTot : 0.0;
      if (total >= 12) {
        int rowNearest = -1;
        for (int y = h - 1; y >= 0; --y)
          if (rowCnt[y] >= 2) {
            rowNearest = y;
            break;
          }
        if (rowNearest >= 0) {
          double depression = std::atan2(rowNearest - horizon, fx);
          if (depression > 0.01) {
            out.waterRange = std::min(15.0, CAM_HEIGHT / std::tan(depression));
            out.waterBearing =
                -std::atan2(static_cast<double>(sumX) / total - w / 2.0, fx);
            out.waterSeen = true;
          }
        }
      }
    }

    // 1) ground reference patch: bottom-centre
    {
      double sr = 0, sg = 0, sb = 0;
      int cnt = 0;
      for (int y = static_cast<int>(h * 0.88); y < h; y += 2)
        for (int x = static_cast<int>(w * 0.4); x < static_cast<int>(w * 0.6);
             x += 2) {
          int r, g, b;
          px(x, y, r, g, b);
          sr += r;
          sg += g;
          sb += b;
          ++cnt;
        }
      if (cnt) {
        double r = sr / cnt, g = sg / cnt, b = sb / cnt;
        if (!isBlue(r, g, b)) {
          if (!haveRef_) {
            ref_[0] = r;
            ref_[1] = g;
            ref_[2] = b;
            haveRef_ = true;
          } else
            for (int k = 0; k < 3; ++k)
              ref_[k] += 0.02 * ((k == 0 ? r : k == 1 ? g : b) - ref_[k]);
        }
      }
    }
    if (!haveRef_) {
      out.valid = true;
      return out;
    }

    // 2) obstacle sectors in the lower-middle band
    int y0 = static_cast<int>(h * 0.55), y1 = static_cast<int>(h * 0.88);
    int cnt[3] = {0, 0, 0}, obs[3] = {0, 0, 0};
    for (int y = y0; y < y1; y += 3)
      for (int x = 0; x < w; x += 3) {
        int sec = std::min(2, x * 3 / w);
        int r, g, b;
        px(x, y, r, g, b);
        ++cnt[sec];
        if (isBlue(r, g, b))
          continue;
        double d = std::sqrt((r - ref_[0]) * (r - ref_[0]) +
                             (g - ref_[1]) * (g - ref_[1]) +
                             (b - ref_[2]) * (b - ref_[2]));
        if (d > COLOR_DIST_THRESH)
          ++obs[sec];
      }
    for (int k = 0; k < 3; ++k)
      out.obstacle[k] = cnt[k] ? static_cast<double>(obs[k]) / cnt[k] : 0.0;
    out.valid = true;
    return out;
  }

private:
  static bool isSky(double r, double g, double b) {
    return b > 225 && g > 175; // bright light blue
  }
  static bool isBlue(double r, double g, double b) {
    return b > 70 && b > r + 20 && b > g + 5;
  }
  static bool isWater(double r, double g, double b) {
    return isBlue(r, g, b) && !isSky(r, g, b);
  }
  static constexpr double COLOR_DIST_THRESH = 70.0;
  static constexpr double CAM_TILT_UP =
      0.15; // camera pitched up by this much (world file); use a negative value
            // for a downward tilt
  static constexpr double CAM_HEIGHT =
      0.25; // camera height above the ground [m]; update after raising the
            // camera in the world file
  double ref_[3] = {0, 0, 0};
  bool haveRef_ = false;
};

// --------------------------------------------------------- stereo depth ----

// Where every candidate pixel of one frame ended up. Every sampled ROI pixel
// is counted exactly once, so
//   candidates == invalid + saturated + texture + range + accepted.
// This is the tool for seeing WHICH gate is rejecting pixels.
struct RejectionFunnel {
  int candidates = 0; // sampled pixels inside the ROI (blind border excluded)
  int invalid = 0;    // SGBM produced no disparity (<= 0 or not finite)
  int saturated = 0;  // disparity at the top of the search range
  int texture = 0;    // rejected: left image too flat (fabricated disparity)
  int range = 0;      // depth outside [minDepth, maxDepth]
  int accepted = 0;   // usable metric depth
};

struct DepthResult {
  double front = 99.0; // unchanged meaning: 99.0 = "no reliable measurement"
  double left = 99.0;
  double right = 99.0;
  double centre = 99.0;
  bool valid = false; // true if at least one sector is `known`

  // Fraction of the sampled ROI pixels in each sector (L, C, R) that produced a
  // usable metric depth. Informational; the obstacle gate is `known`.
  double validFraction[3] = {0.0, 0.0, 0.0};
  // True when the sector holds a CONSISTENT obstacle cluster (>= N pixels whose
  // depths agree within a tolerance). A sector that is not `known` reports 99.0
  // which means "no stereo evidence" -- it does NOT mean "clear". Use lidar for
  // free-space confirmation.
  bool known[3] = {false, false, false};
  // Focal length [px] actually used for this frame.
  double fxUsed = 0.0;

  // ---- diagnostics ----
  RejectionFunnel funnel;  // per-frame rejection counts (see above)
  int fullFrameValid = 0;  // pixels with a valid SGBM disparity, whole image
  bool pipelineOk = false; // SGBM ran and produced matches somewhere
  int roiTop = 0;          // ROI rows actually sampled [roiTop, roiBottom)
  int roiBottom = 0;
  int xStart = 0; // first column sampled (blind border skipped)
};
struct PointXYZRGB {
  float x, y, z;
  unsigned char r, g, b;
};

struct MapPose {
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

// Tunable stereo parameters.
struct StereoParams {
  // --- SGBM matcher ---
  // Disparity search range in pixels (must be a multiple of 16). Sets the
  // closest measurable surface: Zmin = fx*B/(numDisparities-1).
  int numDisparities = 96;
  // Matching window (odd).
  int blockSize = 5;
  // Best match must beat the runner-up by this percentage.
  int uniquenessRatio = 10;
  // Connected disparity blobs smaller than this many pixels are noise.
  int speckleWindowSize = 100;
  // Max disparity jump inside one blob (OpenCV: 1-2 is normal).
  int speckleRange = 2;
  // Left->right->left consistency tolerance in pixels.
  int disp12MaxDiff = 1;
  // Clips the pre-filter output.
  int preFilterCap = 63;

  // --- depth validity (metres) ---
  double minDepth = 0.25;
  // Depth error grows with Z^2; beyond ~6 m this camera pair is not
  // trustworthy.
  double maxDepth = 6.0;

  // --- confidence gating (unchanged strictness) ---
  // Minimum mean |horizontal gradient| in the LEFT image.
  int minTexture = 12;
  // Optional extra gate: min fraction of sampled pixels with a depth. Default 0
  // (off) because in sparse-edge scenes the fraction is naturally tiny even
  // when an obstacle is clearly measured. The cluster gate below replaces it.
  double minValidFraction = 0.0;

  // --- region of interest (milestone 2) ---
  // Rows sampled: [roiTopFraction*h, roiBottomFraction*h). WAS 0.55..0.90,
  // which started BELOW the objects in the test world (their bases were at
  // ~0.545h).
  double roiTopFraction = 0.35;
  double roiBottomFraction = 0.90;
  // SGBM cannot match the first numDisparities columns (minDisparity = 0), so
  // they are excluded from the candidate count instead of counting as failures.
  bool skipBlindBorder = true;

  // --- obstacle evidence gate (milestone 2) ---
  // A sector reports an obstacle only if at least this many accepted pixels lie
  // within `clusterTolerance` (relative) of the same depth. Replaces "10 stray
  // pixels + 10% fraction", which sparse-edge scenes can never satisfy and
  // which stray mismatches could satisfy.
  int minClusterPixels = 15;
  double clusterTolerance = 0.15;

  // Debug visualisation only: disparity (px) that maps to the red end of JET.
  double visMaxDisparity = 32.0;
};

class StereoDepth {
public:
  StereoDepth(double baseline, double fx,
              const StereoParams &params = StereoParams())
      : baseline_(baseline), fx_(fx), params_(params) {
    if (baseline_ <= 0.0 || fx_ <= 0.0)
      throw std::invalid_argument(
          "Stereo baseline and focal length must be positive");
    if (params_.numDisparities <= 0 || params_.numDisparities % 16 != 0)
      throw std::invalid_argument(
          "numDisparities must be a positive multiple of 16");
    if (params_.blockSize < 3 || params_.blockSize % 2 == 0)
      throw std::invalid_argument("blockSize must be odd and >= 3");
    if (params_.minDepth <= 0.0 || params_.maxDepth <= params_.minDepth)
      throw std::invalid_argument("Invalid stereo depth limits");
    if (params_.minValidFraction < 0.0 || params_.minValidFraction > 1.0)
      throw std::invalid_argument("minValidFraction must be in [0, 1]");
    if (params_.speckleWindowSize < 0 || params_.speckleRange < 0 ||
        params_.uniquenessRatio < 0)
      throw std::invalid_argument("Invalid SGBM filtering parameters");
    if (params_.roiTopFraction < 0.0 || params_.roiBottomFraction > 1.0 ||
        params_.roiTopFraction >= params_.roiBottomFraction)
      throw std::invalid_argument("Invalid ROI fractions");
    if (params_.minClusterPixels < 1 || params_.clusterTolerance <= 0.0)
      throw std::invalid_argument("Invalid cluster gate parameters");

    // OpenCV's P1/P2 guidance for a single-channel (grayscale) input:
    // P1 = 8*bs^2, P2 = 32*bs^2.
    const int bs2 = params_.blockSize * params_.blockSize;
    matcher_ = cv::StereoSGBM::create(
        0, params_.numDisparities, params_.blockSize, 8 * bs2, 32 * bs2,
        params_.disp12MaxDiff, params_.preFilterCap, params_.uniquenessRatio,
        params_.speckleWindowSize, params_.speckleRange,
        cv::StereoSGBM::MODE_SGBM_3WAY);
  }

  // fovX: HORIZONTAL field of view [rad] of the real camera (Webots
  // Camera::getFov()). If > 0, the focal length is recomputed from the actual
  // image `width` and `fovX`.
  DepthResult process(const unsigned char *leftBGRA,
                      const unsigned char *rightBGRA, int width, int height,
                      double fovX = 0.0) {
    DepthResult result;

    if (!leftBGRA || !rightBGRA || width < 160 || height < 100 ||
        baseline_ <= 0.0 || fx_ <= 0.0)
      return result;

    double frameFx = fx_;
    if (fovX > 0.0) {
      if (fovX >= PI)
        return result;
      frameFx = (width / 2.0) / std::tan(fovX / 2.0);
    }
    result.fxUsed = frameFx;

    cv::Mat leftImage(height, width, CV_8UC4,
                      const_cast<unsigned char *>(leftBGRA));

    cv::Mat rightImage(height, width, CV_8UC4,
                       const_cast<unsigned char *>(rightBGRA));

    cv::Mat leftGray, rightGray;

    cv::cvtColor(leftImage, leftGray, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(rightImage, rightGray, cv::COLOR_BGRA2GRAY);

    // Calculate disparity (fixed point, 16 sub-pixel steps; invalid = -16).
    cv::Mat disparityFixed;
    matcher_->compute(leftGray, rightGray, disparityFixed);

    disparityFixed.convertTo(disparity_, CV_32F, 1.0 / 16.0);

    // Whole-frame sanity number: is SGBM matching ANYTHING at all?
    result.fullFrameValid = cv::countNonZero(disparity_ > 0.0f);
    result.pipelineOk = result.fullFrameValid > 0;

    // Metric depth map (metres).
    depthMap_ = cv::Mat(height, width, CV_32F,
                        cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    std::vector<double> sectors[3];
    pointCloud_.clear();

    const double cx = (width - 1) / 2.0;
    const double cy = (height - 1) / 2.0;
    const double fy = frameFx; // Webots cameras have square pixels

    const int yStart = static_cast<int>(height * params_.roiTopFraction);
    const int yEnd = static_cast<int>(height * params_.roiBottomFraction);
    const int xStart = params_.skipBlindBorder
                           ? std::min(params_.numDisparities, width - 1)
                           : 0;
    roiTop_ = yStart;
    roiBottom_ = yEnd;
    xStart_ = xStart;
    result.roiTop = yStart;
    result.roiBottom = yEnd;
    result.xStart = xStart;

    // Local texture of the left image, computed only for the rows we use.
    const int r0 = std::max(0, yStart - params_.blockSize);
    const int r1 = std::min(height, yEnd + params_.blockSize);
    cv::Mat gradX, gradAbs, texture;
    cv::Sobel(leftGray.rowRange(r0, r1), gradX, CV_16S, 1, 0, 3);
    cv::convertScaleAbs(gradX, gradAbs);
    cv::boxFilter(gradAbs, texture, CV_8U,
                  cv::Size(params_.blockSize, params_.blockSize));

    // A disparity at the top of the search range is saturated: the true surface
    // may be closer than reported, so it is not a trustworthy measurement.
    const double maxDisparity = params_.numDisparities - 1.0;

    RejectionFunnel &fn = result.funnel;
    int candidates[3] = {0, 0, 0};

    for (int y = yStart; y < yEnd; y += 3) {
      for (int x = xStart; x < width; x += 3) {
        const int sector = std::min(2, x * 3 / width);
        ++candidates[sector];
        ++fn.candidates;

        const float d = disparity_.at<float>(y, x);

        if (!std::isfinite(d) || d <= 0.0f) {
          ++fn.invalid;
          continue;
        }
        if (d >= maxDisparity) {
          ++fn.saturated;
          continue;
        }

        if (texture.at<unsigned char>(y - r0, x) < params_.minTexture) {
          ++fn.texture;
          continue;
        }

        const double distance = (frameFx * baseline_) / d;

        if (!std::isfinite(distance) || distance < params_.minDepth ||
            distance > params_.maxDepth) {
          ++fn.range;
          continue;
        }

        ++fn.accepted;
        depthMap_.at<float>(y, x) = static_cast<float>(distance);

        // Reconstruct 3D coordinates in the camera frame.
        const double X = (x - cx) * distance / frameFx;
        const double Y = (y - cy) * distance / fy;
        const double Z = distance;

        // Get colour from the left camera image (BGRA format).
        const unsigned char *pixel =
            leftBGRA + 4 * (static_cast<size_t>(y) * width + x);

        PointXYZRGB point;
        point.x = static_cast<float>(X);
        point.y = static_cast<float>(Y);
        point.z = static_cast<float>(Z);
        point.r = pixel[2];
        point.g = pixel[1];
        point.b = pixel[0];

        pointCloud_.push_back(point);

        sectors[sector].push_back(distance);
      }
    }

    // Obstacle-evidence gate: the nearest depth D such that at least
    // minClusterPixels accepted pixels lie in [D, D*(1+tol)]. Isolated stray
    // matches cannot satisfy it; a real object (even one visible only through
    // its edges) can. Invalid / untextured pixels are never accepted.
    auto sectorDistance = [&](int k) {
      std::vector<double> &values = sectors[k];
      result.validFraction[k] =
          candidates[k] ? static_cast<double>(values.size()) / candidates[k]
                        : 0.0;
      result.known[k] = false;

      const size_t need = static_cast<size_t>(params_.minClusterPixels);
      if (values.size() < need ||
          result.validFraction[k] < params_.minValidFraction)
        return 99.0; // NO EVIDENCE (not "clear")

      std::sort(values.begin(), values.end());
      size_t j = 0;
      for (size_t i = 0; i + need <= values.size(); ++i) {
        if (j < i)
          j = i;
        const double limit = values[i] * (1.0 + params_.clusterTolerance);
        while (j < values.size() && values[j] <= limit)
          ++j;
        if (j - i >= need) {
          result.known[k] = true;
          return values[i];
        }
      }
      return 99.0;
    };

    result.left = sectorDistance(0);
    result.centre = sectorDistance(1);
    result.right = sectorDistance(2);

    result.front = result.centre;

    result.valid = result.known[0] || result.known[1] || result.known[2];

    return result;
  }

  // One-line summary of what this geometry can and cannot resolve.
  std::string describe(int width, int height) const {
    char buf[320];
    const double fb = fx_ * baseline_;
    std::snprintf(buf, sizeof(buf),
                  "%dx%d fx=%.1fpx baseline=%.3fm numDisp=%d | Zmin=%.2fm "
                  "Zmax(cfg)=%.1fm | "
                  "depth error for 0.25px disparity error: 1m->%.2f 2m->%.2f "
                  "3m->%.2f 5m->%.2f m",
                  width, height, fx_, baseline_, params_.numDisparities,
                  fb / (params_.numDisparities - 1.0), params_.maxDepth,
                  1.0 / fb * 0.25, 4.0 / fb * 0.25, 9.0 / fb * 0.25,
                  25.0 / fb * 0.25);
    return std::string(buf);
  }

  // One-line rejection funnel + full-frame sanity numbers for the log.
  static std::string funnelSummary(const DepthResult &r) {
    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "full-frame valid disp=%d | ROI rows %d-%d, x>=%d | "
                  "cand=%d invalid=%d saturated=%d flat=%d range=%d "
                  "accepted=%d | known L/C/R=%d/%d/%d",
                  r.fullFrameValid, r.roiTop, r.roiBottom, r.xStart,
                  r.funnel.candidates, r.funnel.invalid, r.funnel.saturated,
                  r.funnel.texture, r.funnel.range, r.funnel.accepted,
                  r.known[0] ? 1 : 0, r.known[1] ? 1 : 0, r.known[2] ? 1 : 0);
    return std::string(buf);
  }

  // Raw SGBM disparity at one pixel (px). NaN if out of bounds / no frame yet.
  // Use this to check that known edges produce the expected disparity.
  float disparityAt(int y, int x) const {
    if (disparity_.empty() || y < 0 || x < 0 || y >= disparity_.rows ||
        x >= disparity_.cols)
      return std::numeric_limits<float>::quiet_NaN();
    return disparity_.at<float>(y, x);
  }

  // Builds the colour visualisations on demand. Invalid pixels are BLACK so
  // they can no longer be mistaken for a valid disparity of 0 (dark blue). The
  // sampled ROI is drawn on left.png.
  bool saveDebugImages(const std::string &directory,
                       const unsigned char *leftBGRA,
                       const unsigned char *rightBGRA, int width, int height) {

    prepareVisualisations();

    if (!leftBGRA || !rightBGRA || disparityVis_.empty() || depthVis_.empty())
      return false;

    std::error_code ec;
    std::filesystem::create_directories(directory, ec);

    if (ec)
      return false;

    cv::Mat left(height, width, CV_8UC4, const_cast<unsigned char *>(leftBGRA));

    cv::Mat right(height, width, CV_8UC4,
                  const_cast<unsigned char *>(rightBGRA));

    cv::Mat leftBGR, rightBGR;

    cv::cvtColor(left, leftBGR, cv::COLOR_BGRA2BGR);
    cv::cvtColor(right, rightBGR, cv::COLOR_BGRA2BGR);

    cv::Mat leftRoi = leftBGR.clone();
    if (roiBottom_ > roiTop_)
      cv::rectangle(leftRoi, cv::Point(xStart_, roiTop_),
                    cv::Point(width - 1, roiBottom_ - 1), cv::Scalar(0, 0, 255),
                    1);

    try {
      const bool a = cv::imwrite(directory + "/left.png", leftBGR);
      const bool b = cv::imwrite(directory + "/right.png", rightBGR);
      const bool c = cv::imwrite(directory + "/disparity.png", disparityVis_);
      const bool d = cv::imwrite(directory + "/depth.png", depthVis_);
      const bool e = cv::imwrite(directory + "/left_roi.png", leftRoi);

      return a && b && c && d && e;
    } catch (const cv::Exception &) {
      return false;
    }
  }

public:
  bool savePointCloudPLY(const std::string &filename) const {
    if (pointCloud_.empty())
      return false;

    std::ofstream file(filename);
    if (!file.is_open())
      return false;

    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << pointCloud_.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    for (const auto &p : pointCloud_) {
      file << p.x << " " << p.y << " " << p.z << " " << static_cast<int>(p.r)
           << " " << static_cast<int>(p.g) << " " << static_cast<int>(p.b)
           << "\n";
    }

    return true;
  }

private:
public:
  void accumulate(const MapPose &pose) {
    const double cr = std::cos(pose.roll);
    const double sr = std::sin(pose.roll);
    const double cp = std::cos(pose.pitch);
    const double sp = std::sin(pose.pitch);
    const double cy = std::cos(pose.yaw);
    const double sy = std::sin(pose.yaw);

    constexpr size_t MAX_MAP_POINTS = 300000;

    // Camera mounting translation in rover body coordinates.
    // Based on the left Webots camera translation.
    constexpr double CAMERA_X = 0.30;
    constexpr double CAMERA_Y = 0.03;
    constexpr double CAMERA_Z = 0.15;

    // Keep only the current frame here; worldMap_ remains cumulative.
    latestWorldCloud_.clear();
    latestWorldCloud_.reserve(pointCloud_.size());

    for (const auto &p : pointCloud_) {
      // Camera optical coordinates: X right, Y down, Z forward.
      double rx = p.z + CAMERA_X;
      double ry = -p.x + CAMERA_Y;
      double rz = -p.y + CAMERA_Z;

      // Rover roll.
      double x1 = rx;
      double y1 = cr * ry - sr * rz;
      double z1 = sr * ry + cr * rz;

      // Rover pitch.
      double x2 = cp * x1 + sp * z1;
      double y2 = y1;
      double z2 = -sp * x1 + cp * z1;

      // Rover yaw and world translation.
      PointXYZRGB q = p;
      q.x = static_cast<float>(cy * x2 - sy * y2 + pose.x);
      q.y = static_cast<float>(sy * x2 + cy * y2 + pose.y);
      q.z = static_cast<float>(z2 + pose.z);

      latestWorldCloud_.push_back(q);

      if (worldMap_.size() < MAX_MAP_POINTS)
        worldMap_.push_back(q);
    }
  }

  const std::vector<PointXYZRGB> &latestWorldCloud() const {
    return latestWorldCloud_;
  }

  bool saveMapPLY(const std::string &filename) const {
    if (worldMap_.empty())
      return false;

    std::ofstream file(filename);
    if (!file.is_open())
      return false;

    file << "ply\nformat ascii 1.0\n";
    file << "element vertex " << worldMap_.size() << "\n";
    file << "property float x\nproperty float y\nproperty float z\n";
    file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    file << "end_header\n";

    for (const auto &p : worldMap_) {
      file << p.x << " " << p.y << " " << p.z << " " << static_cast<int>(p.r)
           << " " << static_cast<int>(p.g) << " " << static_cast<int>(p.b)
           << "\n";
    }

    return file.good();
  }

  size_t mapSize() const { return worldMap_.size(); }

  double baseline_;
  double fx_;
  StereoParams params_;

  cv::Ptr<cv::StereoSGBM> matcher_;

  cv::Mat disparity_;
  cv::Mat depthMap_;

  cv::Mat disparityVis_;
  cv::Mat depthVis_;
  std::vector<PointXYZRGB> pointCloud_;
  std::vector<PointXYZRGB> latestWorldCloud_;
  std::vector<PointXYZRGB> worldMap_;

  int roiTop_ = 0, roiBottom_ = 0, xStart_ = 0;

  void prepareVisualisations() {
    if (disparity_.empty() || depthMap_.empty())
      return;

    // Disparity: scaled to [0, visMaxDisparity] px so real values (a few to a
    // few tens of px) use the whole colour range; INVALID pixels are black.
    cv::Mat disparity8;
    disparity_.convertTo(disparity8, CV_8U, 255.0 / params_.visMaxDisparity);
    cv::applyColorMap(disparity8, disparityVis_, cv::COLORMAP_JET);
    disparityVis_.setTo(cv::Scalar(0, 0, 0), disparity_ <= 0.0f);

    // Depth: near = red, far = blue over [minDepth, maxDepth]; pixels with no
    // accepted depth are black.
    cv::Mat depth8(depthMap_.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat noDepth(depthMap_.size(), CV_8UC1, cv::Scalar(255));
    const float zMin = static_cast<float>(params_.minDepth);
    const float zMax = static_cast<float>(params_.maxDepth);

    for (int y = 0; y < depthMap_.rows; ++y) {
      const float *src = depthMap_.ptr<float>(y);
      unsigned char *dst = depth8.ptr<unsigned char>(y);
      unsigned char *nd = noDepth.ptr<unsigned char>(y);
      for (int x = 0; x < depthMap_.cols; ++x) {
        const float z = src[x];

        if (!std::isfinite(z))
          continue;

        nd[x] = 0;
        const float value = (zMax - z) / (zMax - zMin);

        dst[x] =
            static_cast<unsigned char>(std::clamp(value, 0.0f, 1.0f) * 255.0f);
      }
    }

    cv::applyColorMap(depth8, depthVis_, cv::COLORMAP_JET);
    depthVis_.setTo(cv::Scalar(0, 0, 0), noDepth);
  }
};
} // namespace rover
