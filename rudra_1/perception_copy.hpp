#pragma once
// Lidar scan summary + simple camera-based obstacle / water detector.
// Pure C++, no Webots types: the controller passes raw arrays in.
#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>
#include <stdexcept>
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

struct DepthResult {
  double front = 99.0;
  double left = 99.0;
  double right = 99.0;
  double centre = 99.0;
  bool valid = false;
};
struct PointXYZRGB {
  float x, y, z;
  unsigned char r, g, b;
};

struct MapPose {
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

class StereoDepth {
public:
  StereoDepth(double baseline, double fx) : baseline_(baseline), fx_(fx) {
    if (baseline_ <= 0.0 || fx_ <= 0.0)
      throw std::invalid_argument("Stereo baseline and focal length must be positive");

    matcher_ =
        cv::StereoSGBM::create(0, 96, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 63, 10, 100,
                               32, cv::StereoSGBM::MODE_SGBM_3WAY);
  }

  DepthResult process(const unsigned char *leftBGRA,
                      const unsigned char *rightBGRA, int width, int height) {
    DepthResult result;

    if (!leftBGRA || !rightBGRA || width < 160 || height < 100 ||
        baseline_ <= 0.0 || fx_ <= 0.0)
      return result;

    cv::Mat leftImage(height, width, CV_8UC4,
                      const_cast<unsigned char *>(leftBGRA));

    cv::Mat rightImage(height, width, CV_8UC4,
                       const_cast<unsigned char *>(rightBGRA));

    cv::Mat leftGray, rightGray;

    cv::cvtColor(leftImage, leftGray, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(rightImage, rightGray, cv::COLOR_BGRA2GRAY);

    // Calculate disparity.
    cv::Mat disparityFixed;
    matcher_->compute(leftGray, rightGray, disparityFixed);

    disparityFixed.convertTo(disparity_, CV_32F, 1.0 / 16.0);

    // Metric depth map (metres).
    depthMap_ = cv::Mat(height, width, CV_32F,
                        cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    std::vector<double> sectors[3];
    pointCloud_.clear();

    const double cx = (width - 1) / 2.0;
    const double cy = (height - 1) / 2.0;
    const double fy = fx_;

    const int yStart = static_cast<int>(height * 0.55);
    const int yEnd = static_cast<int>(height * 0.90);

    for (int y = yStart; y < yEnd; y += 3) {
      for (int x = 0; x < width; x += 3) {
        const float d = disparity_.at<float>(y, x);

        if (!std::isfinite(d) || d <= 1.0f)
          continue;

        const double distance = (fx_ * baseline_) / d;

        if (!std::isfinite(distance) || distance < 0.25 || distance > 10.0)
          continue;

        depthMap_.at<float>(y, x) = static_cast<float>(distance);

        // Reconstruct 3D coordinates in the camera frame.
        const double X = (x - cx) * distance / fx_;
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

        // Keep the existing sector-distance calculation.
        const int sector = std::min(2, x * 3 / width);
        sectors[sector].push_back(distance);
      }
    }

    auto obstacleDistance = [](std::vector<double> &values) {
      if (values.size() < 10)
        return 99.0;

      // 25th percentile: more responsive to nearby obstacles than
      // the median, while being less sensitive to isolated bad pixels.
      const size_t index = values.size() / 4;

      std::nth_element(values.begin(), values.begin() + index, values.end());

      return values[index];
    };

    result.left = obstacleDistance(sectors[0]);
    result.centre = obstacleDistance(sectors[1]);
    result.right = obstacleDistance(sectors[2]);

    result.front = result.centre;

    result.valid =
        result.left < 99.0 || result.centre < 99.0 || result.right < 99.0;

    prepareVisualisations();

    return result;
  }

  bool saveDebugImages(const std::string &directory,
                       const unsigned char *leftBGRA,
                       const unsigned char *rightBGRA, int width,
                       int height) const {

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

    try {
      const bool a = cv::imwrite(directory + "/left.png", leftBGR);
      const bool b = cv::imwrite(directory + "/right.png", rightBGR);
      const bool c = cv::imwrite(directory + "/disparity.png", disparityVis_);
      const bool d = cv::imwrite(directory + "/depth.png", depthVis_);

      return a && b && c && d;
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

       for (const auto &p : pointCloud_) {
            if (worldMap_.size() >= MAX_MAP_POINTS)
                break;

            // Camera: X right, Y down, Z forward.
            double rx = p.z;
            double ry = -p.x;
            double rz = -p.y;

            double x1 = rx;
            double y1 = cr * ry - sr * rz;
            double z1 = sr * ry + cr * rz;

            double x2 = cp * x1 + sp * z1;
            double y2 = y1;
            double z2 = -sp * x1 + cp * z1;

            PointXYZRGB q = p;
            q.x = static_cast<float>(cy * x2 - sy * y2 + pose.x);
            q.y = static_cast<float>(sy * x2 + cy * y2 + pose.y);
            q.z = static_cast<float>(z2 + pose.z);

            worldMap_.push_back(q);
        }
    }

    bool saveMapPLY(const std::string &filename) const {
        if (worldMap_.empty()) return false;

        std::ofstream file(filename);
        if (!file.is_open()) return false;

        file << "ply\nformat ascii 1.0\n";
        file << "element vertex " << worldMap_.size() << "\n";
        file << "property float x\nproperty float y\nproperty float z\n";
        file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
        file << "end_header\n";

        for (const auto &p : worldMap_) {
            file << p.x << " " << p.y << " " << p.z << " "
                 << static_cast<int>(p.r) << " "
                 << static_cast<int>(p.g) << " "
                 << static_cast<int>(p.b) << "\n";
        }

        return file.good();
    }

    size_t mapSize() const {
        return worldMap_.size();
    }


  double baseline_;
  double fx_;

  cv::Ptr<cv::StereoSGBM> matcher_;

  cv::Mat disparity_;
  cv::Mat depthMap_;

  cv::Mat disparityVis_;
  cv::Mat depthVis_;
  std::vector<PointXYZRGB> pointCloud_;
    std::vector<PointXYZRGB> worldMap_;

  void prepareVisualisations() {
    if (disparity_.empty() || depthMap_.empty())
      return;

    // Disparity visualisation.
    cv::Mat disparity8;

    disparity_.convertTo(disparity8, CV_8U, 255.0 / 96.0);

    cv::applyColorMap(disparity8, disparityVis_, cv::COLORMAP_JET);

    // Depth visualisation.
    cv::Mat depth8(depthMap_.size(), CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < depthMap_.rows; ++y) {
      for (int x = 0; x < depthMap_.cols; ++x) {
        const float z = depthMap_.at<float>(y, x);

        if (!std::isfinite(z))
          continue;

        const float value = (10.0f - z) / (10.0f - 0.25f);

        depth8.at<unsigned char>(y, x) =
            static_cast<unsigned char>(std::clamp(value, 0.0f, 1.0f) * 255.0f);
      }
    }

    cv::applyColorMap(depth8, depthVis_, cv::COLORMAP_JET);
  }
};
} // namespace rover
