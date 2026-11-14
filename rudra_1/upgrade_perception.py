from pathlib import Path
import shutil

p = Path("perception.hpp")
s = p.read_text()

backup = Path("perception.hpp.before_ground_upgrade")
if not backup.exists():
    shutil.copy2(p, backup)

def replace(old, new, label):
    global s
    count = s.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected 1 match, found {count}")
    s = s.replace(old, new, 1)

# Additional headers for the voxel map.
replace(
    "#include <limits>",
    "#include <limits>\n#include <map>\n#include <tuple>",
    "Header insertion"
)

# Ground-plane parameters.
replace(
    " double minValidFraction = 0.10;\n};",
    """ double minValidFraction = 0.10;

 // --- RANSAC ground-plane removal ---
 int groundRansacIterations = 120;
 double groundInlierThreshold = 0.06; // metres
 double groundNormalYMin = 0.65;
 int minGroundInliers = 30;

 // --- Voxel map ---
 double mapVoxelSize = 0.10; // metres
 size_t maxMapVoxels = 300000;
};""",
    "StereoParams insertion"
)

# Validate new parameters.
replace(
    '   if (params_.speckleWindowSize < 0 ||\n'
    '       params_.speckleRange < 0 ||\n'
    '       params_.uniquenessRatio < 0)\n'
    '     throw std::invalid_argument("Invalid SGBM filtering parameters");',
    """   if (params_.speckleWindowSize < 0 ||
       params_.speckleRange < 0 ||
       params_.uniquenessRatio < 0)
     throw std::invalid_argument("Invalid SGBM filtering parameters");
   if (params_.groundRansacIterations < 1 ||
       params_.groundInlierThreshold <= 0.0 ||
       params_.groundNormalYMin <= 0.0 ||
       params_.groundNormalYMin > 1.0 ||
       params_.minGroundInliers < 3)
     throw std::invalid_argument("Invalid ground RANSAC parameters");
   if (params_.mapVoxelSize <= 0.0 || params_.maxMapVoxels == 0)
     throw std::invalid_argument("Invalid voxel map parameters");""",
    "Ground/map validation"
)

# Do not calculate sector distances while the points are still unfiltered.
replace(
    "       pointCloud_.push_back(point);\n\n"
    "       sectors[sector].push_back(distance);",
    "       pointCloud_.push_back(point);",
    "Defer sector distances"
)

# Replace sector aggregation with ground-plane RANSAC and obstacle-only sectors.
old = """   auto sectorDistance = [&](int k) {
     std::vector<double> &values = sectors[k];
     result.validFraction[k] =
         candidates[k] ? static_cast<double>(values.size()) / candidates[k]
                       : 0.0;
     result.known[k] = values.size() >= 10 &&
                       result.validFraction[k] >= params_.minValidFraction;
     if (!result.known[k])
       return 99.0; // UNKNOWN (not "clear"): check `known` before trusting it

     // 25th percentile, unchanged in this milestone. NOTE: the ROI still
     // contains ground pixels, so this is "near-field depth", not yet a pure
     // obstacle distance. Ground handling is the next milestone; lowering the
     // percentile before that would only make ground trigger the stop
     // distance.
     const size_t index = values.size() / 4;

     std::nth_element(values.begin(), values.begin() + index, values.end());

     return values[index];
   };"""

new = """   // Fit the ground plane in camera coordinates:
   // n.x*X + n.y*Y + n.z*Z + d = 0.
   // The camera Y axis points down, so a ground plane should have a
   // substantial Y component in its normal.
   bool groundFound = false;
   double bestA = 0.0, bestB = 0.0, bestC = 0.0, bestD = 0.0;
   size_t bestInliers = 0;

   const size_t pointCount = pointCloud_.size();
   if (pointCount >= static_cast<size_t>(params_.minGroundInliers)) {
     uint32_t rng = 0x9e3779b9u;
     auto nextIndex = [&]() -> size_t {
       rng ^= rng << 13;
       rng ^= rng >> 17;
       rng ^= rng << 5;
       return static_cast<size_t>(rng) % pointCount;
     };

     for (int iter = 0; iter < params_.groundRansacIterations; ++iter) {
       size_t i1 = nextIndex(), i2 = nextIndex(), i3 = nextIndex();
       if (i1 == i2 || i1 == i3 || i2 == i3)
         continue;

       const auto &p1 = pointCloud_[i1];
       const auto &p2 = pointCloud_[i2];
       const auto &p3 = pointCloud_[i3];

       const double ux = p2.x - p1.x;
       const double uy = p2.y - p1.y;
       const double uz = p2.z - p1.z;
       const double vx = p3.x - p1.x;
       const double vy = p3.y - p1.y;
       const double vz = p3.z - p1.z;

       double a = uy * vz - uz * vy;
       double b = uz * vx - ux * vz;
       double c = ux * vy - uy * vx;
       const double norm = std::sqrt(a*a + b*b + c*c);

       if (norm < 1e-8)
         continue;

       a /= norm;
       b /= norm;
       c /= norm;

       if (std::fabs(b) < params_.groundNormalYMin)
         continue;

       const double d0 = -(a*p1.x + b*p1.y + c*p1.z);
       size_t inliers = 0;

       for (const auto &q : pointCloud_) {
         const double error =
             std::fabs(a*q.x + b*q.y + c*q.z + d0);
         if (error <= params_.groundInlierThreshold)
           ++inliers;
       }

       if (inliers > bestInliers) {
         bestInliers = inliers;
         bestA = a;
         bestB = b;
         bestC = c;
         bestD = d0;
       }
     }

     groundFound =
         bestInliers >= static_cast<size_t>(params_.minGroundInliers);
   }

   // If no plausible ground plane is found, do not guess that the ground
   // is an obstacle-free surface. All sectors remain UNKNOWN.
   std::vector<PointXYZRGB> obstaclePoints;
   if (groundFound) {
     obstaclePoints.reserve(pointCloud_.size());

     for (const auto &p : pointCloud_) {
       const double error =
           std::fabs(bestA*p.x + bestB*p.y + bestC*p.z + bestD);

       if (error <= params_.groundInlierThreshold)
         continue;

       obstaclePoints.push_back(p);

       // Project the point back into the image to identify its sector.
       const double imageX = frameFx * p.x / p.z + cx;
       const int sector = std::max(
           0, std::min(2, static_cast<int>(imageX * 3.0 / width)));

       sectors[sector].push_back(p.z);
     }
   }

   // The exported current-frame cloud now contains obstacle points only.
   pointCloud_.swap(obstaclePoints);

   auto sectorDistance = [&](int k) {
     std::vector<double> &values = sectors[k];

     result.validFraction[k] =
         candidates[k]
             ? static_cast<double>(values.size()) / candidates[k]
             : 0.0;

     result.known[k] =
         groundFound && values.size() >= 10 &&
         result.validFraction[k] >= params_.minValidFraction;

     if (!result.known[k])
       return 99.0;

     // Use the 20th percentile of obstacle points to remain sensitive
     // to nearby obstacles without relying on a single noisy pixel.
     const size_t index = values.size() / 5;
     std::nth_element(values.begin(), values.begin() + index, values.end());
     return values[index];
   };"""

replace(old, new, "RANSAC sector replacement")

# Replace append-only world map with a bounded voxel map.
replace(
    """   constexpr size_t MAX_MAP_POINTS = 300000;

   for (const auto &p : pointCloud_) {
     if (worldMap_.size() >= MAX_MAP_POINTS)
       break;

     // Camera: X right, Y down, Z forward.""",
    """   for (const auto &p : pointCloud_) {
     // Camera: X right, Y down, Z forward.""",
    "Remove append-only map limit"
)

replace(
    """     worldMap_.push_back(q);
   }
 }""",
    """     const double voxel = params_.mapVoxelSize;
     const auto key = std::make_tuple(
         static_cast<int>(std::floor(q.x / voxel)),
         static_cast<int>(std::floor(q.y / voxel)),
         static_cast<int>(std::floor(q.z / voxel)));

     // Existing voxels are updated; new voxels are added only while
     // below the configured memory limit.
     auto it = voxelMap_.find(key);
     if (it != voxelMap_.end()) {
       it->second = q;
     } else if (voxelMap_.size() < params_.maxMapVoxels) {
       voxelMap_.emplace(key, q);
     }
   }
 }""",
    "Voxel accumulation"
)

# Save the voxel map directly.
replace(
    """   if (worldMap_.empty())
     return false;""",
    """   if (voxelMap_.empty())
     return false;""",
    "Map empty check"
)

replace(
    """   file << "element vertex " << worldMap_.size() << "\\\\n";""",
    """   file << "element vertex " << voxelMap_.size() << "\\\\n";""",
    "Map PLY vertex count"
)

replace(
    """   for (const auto &p : worldMap_) {
     file << p.x << " " << p.y << " " << p.z << " " << static_cast<int>(p.r)
          << " " << static_cast<int>(p.g) << " " << static_cast<int>(p.b)
          << "\\\\n";
   }

   return file.good();""",
    """   for (const auto &entry : voxelMap_) {
     const auto &p = entry.second;
     file << p.x << " " << p.y << " " << p.z << " " << static_cast<int>(p.r)
          << " " << static_cast<int>(p.g) << " " << static_cast<int>(p.b)
          << "\\\\n";
   }

   return file.good();""",
    "Voxel map PLY output"
)

replace(
    " size_t mapSize() const { return worldMap_.size(); }",
    " size_t mapSize() const { return voxelMap_.size(); }",
    "Map size"
)

replace(
    " std::vector<PointXYZRGB> worldMap_;",
    """ std::map<std::tuple<int, int, int>, PointXYZRGB> voxelMap_;""",
    "Replace world map storage"
)

# Write only after every expected change has matched.
p.write_text(s)
print("Perception upgrade applied successfully.")
print(f"Backup: {backup}")
