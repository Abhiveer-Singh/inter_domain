#ifndef ROVER_CONFIG_HPP
#define ROVER_CONFIG_HPP

#include <array>

#include "utils.hpp"

namespace rover::config {

// =============================================================
// HARDWARE CONFIGURATION
// =============================================================

inline constexpr int WHEEL_COUNT = 6;

inline constexpr std::array<const char *, WHEEL_COUNT> WHEEL_NAMES = {
    "front_left_wheel_motor",   "middle_left_wheel_motor",
    "rear_left_wheel_motor",    "front_right_wheel_motor",
    "middle_right_wheel_motor", "rear_right_wheel_motor"};

inline constexpr const char *IMU_NAME = "imu";
inline constexpr const char *GYRO_NAME = "gyro";
inline constexpr const char *ACCEL_NAME = "accelerometer";
inline constexpr const char *LIDAR_NAME = "lidar";
inline constexpr const char *CAMERA_NAME = "camera";
inline constexpr const char *GPS_NAME = "gps";

// =============================================================
// ROVER GEOMETRY
// =============================================================

inline constexpr double WHEEL_RADIUS = 0.10;
inline constexpr double TRACK_WIDTH = 0.45;
inline constexpr double SLIP_FACTOR = 1.20;

inline constexpr double ROBOT_RADIUS = 0.30;
inline constexpr double INFLATION_RADIUS = 0.70;

// =============================================================
// MOTION LIMITS
// =============================================================

inline constexpr double MAX_WHEEL_RAD_S = 8.0;
inline constexpr double MAX_SPEED = 0.45;

// =============================================================
// OBSTACLE AVOIDANCE
// =============================================================

inline constexpr double OBSTACLE_STOP_DISTANCE = 0.55;
inline constexpr double OBSTACLE_SLOW_DISTANCE = 1.20;
inline constexpr double MAX_SENSOR_RANGE = 10.0;

// =============================================================
// NAVIGATION
// =============================================================

inline constexpr double GOAL_TOLERANCE = 0.35;
inline constexpr double REPLAN_INTERVAL = 1.0;

// Occupancy grid dimensions.
inline constexpr double MAP_SIZE = 40.0;
inline constexpr double MAP_RESOLUTION = 0.20;

// =============================================================
// STATIC WATER-BODY MAP
// =============================================================

// Example values. Replace them with the actual geometry
// of the water bodies in your Webots world.

inline constexpr double WATER_RADIUS = 1.0;

inline constexpr bool WATER_MAP_CONFIGURED = false;

// Example safe destinations beside the water bodies.
// These are placeholders, not verified world coordinates.

inline const std::array<Vec2, 2> WATER_GOALS = {{{-4.0, 3.0}, {5.0, -2.0}}};

// Example water-body centres.
inline constexpr Vec2 WATER_CENTRES[2] = {{-5.0, 4.0}, {5.0, -3.0}};

// =============================================================
// INITIAL LOCALIZATION
// =============================================================

inline constexpr double INITIAL_X = 0.0;
inline constexpr double INITIAL_Y = 0.0;
inline constexpr double INITIAL_YAW = 0.0;

} // namespace rover::config

#endif // ROVER_CONFIG_HPP
