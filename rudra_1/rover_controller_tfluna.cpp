// TF-Luna pan/tilt range sensing -> local occupancy grid -> A* navigation
// IMU/GPS EKF -> pure pursuit -> reactive avoidance -> PID -> skid steer.

#include <webots/Accelerometer.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "astar.hpp"
#include "config.hpp"
#include "ekf.hpp"
#include "filters.hpp"
#include "occupancy_grid.hpp"
#include "pid.hpp"
#include "skid_steer.hpp"
#include "state_machine.hpp"
#include "tfluna_perception.hpp"
#include "utils.hpp"

using namespace rover;

// =============================================================== CONFIG ====
namespace cfg {
inline const std::vector<std::string> LEFT_WHEELS = {
    "left_motor_1", "left_motor_2", "left_motor_3"};
inline const std::vector<std::string> RIGHT_WHEELS = {
    "right_motor_1", "right_motor_2", "right_motor_3"};
constexpr const char *IMU_NAME = "imu";
constexpr const char *GYRO_NAME = "gyro";
constexpr const char *ACCEL_NAME = "accelerometer";
constexpr const char *GPS_NAME = "gps";
constexpr const char *TF_LUNA_NAME = "tf_luna";
constexpr const char *PAN_MOTOR_NAME = "lidar_pan_motor";
constexpr const char *TILT_MOTOR_NAME = "lidar_tilt_motor";
constexpr double TF_MIN_RANGE = 0.20;
constexpr double TF_MAX_RANGE = 12.0;
constexpr double TF_PAN_MIN = -1.30;
constexpr double TF_PAN_MAX = 1.30;
constexpr double TF_PAN_SPEED = 2.0;
constexpr double TF_SAMPLE_PERIOD = 0.04;
constexpr double TF_SAMPLE_ANGLE_STEP = 0.035;
constexpr double TF_SCAN_STALE_S = 3.0;
constexpr double TF_SENSOR_X = 0.34;
constexpr double TF_SENSOR_Y = 0.0;

constexpr double LEFT_DIR = 1.0;
constexpr double RIGHT_DIR = 1.0;

constexpr double WHEEL_RADIUS = 0.06;
constexpr double TRACK_WIDTH = 0.34;
constexpr double SLIP_FACTOR = 1.4;
constexpr double MAX_SPEED =
    0.40; // conservative for a mechanically swept single-beam sensor
constexpr double MAX_YAW_RATE = 1.0;
constexpr double MAX_ACCEL = 0.6;
constexpr double MAX_DECEL = 1.5;
constexpr double ROBOT_RADIUS = 0.45;
constexpr double INFLATION_RADIUS = 1.0;

constexpr double MAP_SIZE = 100.0;
constexpr double MAP_RES = 0.25;

struct Water {
  double x, y, radius;
};
inline std::vector<Water> WATER;

constexpr bool USE_WATER_HINTS = false;
inline const std::vector<Water> WATER_HINTS = {{0.0, -10.0, 5.0},
                                               {-8.0, 9.0, 5.0}};
constexpr double STANDOFF = 1.5;
constexpr double ARRIVE_TOL = 0.7;
constexpr double WAIT_AT_WATER_S = 4.0;
constexpr int NUM_WATER_BODIES = 2;
constexpr double HOME_TOL = 0.5;
constexpr double POND_AVOID_RADIUS = 3.5;

constexpr double START_X = 0.0, START_Y = 0.0;

constexpr double LOOKAHEAD = 1.2;
constexpr double REPLAN_PERIOD = 1.0;
constexpr double STOP_DIST = 0.9;
constexpr double SLOW_DIST = 2.5;
constexpr double CLEAR_DIST = 2.0;
constexpr double ARENA_HALF = 22.0;
constexpr double POND_RADIUS_GUESS = 3.0;
constexpr double SAME_POND = 2.5;
constexpr double SAME_POND_VISITED = 9.0;
constexpr double INSPECTED_EXCLUDE = 10.0;
constexpr double SPIN_TIMEOUT_S = 8.0;
constexpr double STALL_TIME_S = 3.0;
constexpr double RECOVER_TIME_S = 3.0;
constexpr double RECOVER_SPEED = 0.30;
constexpr double ESCAPE_TIME_S = 3.0;
constexpr double ESCAPE_SPEED = 0.35;
} // namespace cfg

// ================================================================ MAIN ====
int main() {
  webots::Robot robot;
  const int ts = static_cast<int>(robot.getBasicTimeStep());
  const double dt = ts / 1000.0;

  // ---- wheels ----
  std::vector<webots::Motor *> leftM, rightM;
  std::vector<webots::PositionSensor *> leftPS, rightPS;
  auto setupWheels = [&](const std::vector<std::string> &names,
                         std::vector<webots::Motor *> &m,
                         std::vector<webots::PositionSensor *> &ps) {
    for (const auto &n : names) {
      webots::Motor *mo = robot.getMotor(n);
      if (!mo) {
        std::printf("[init] motor '%s' not found\n", n.c_str());
        continue;
      }
      mo->setPosition(INFINITY);
      mo->setVelocity(0.0);
      m.push_back(mo);
      webots::PositionSensor *p = mo->getPositionSensor();
      if (p)
        p->enable(ts);
      ps.push_back(p);
    }
  };
  setupWheels(cfg::LEFT_WHEELS, leftM, leftPS);
  setupWheels(cfg::RIGHT_WHEELS, rightM, rightPS);
  double maxWheelRadS = 1e9;
  for (auto *m : leftM)
    maxWheelRadS = std::min(maxWheelRadS, m->getMaxVelocity());
  for (auto *m : rightM)
    maxWheelRadS = std::min(maxWheelRadS, m->getMaxVelocity());
  if (maxWheelRadS <= 0.0 || maxWheelRadS > 1e8)
    maxWheelRadS = cfg::MAX_SPEED / cfg::WHEEL_RADIUS;

  // ---- sensors ----
  webots::InertialUnit *imu = robot.getInertialUnit(cfg::IMU_NAME);
  webots::Gyro *gyro = robot.getGyro(cfg::GYRO_NAME);
  webots::Accelerometer *accel = robot.getAccelerometer(cfg::ACCEL_NAME);
  webots::GPS *gps = robot.getGPS(cfg::GPS_NAME);
  webots::DistanceSensor *tfLuna = robot.getDistanceSensor(cfg::TF_LUNA_NAME);
  webots::Motor *panMotor = robot.getMotor(cfg::PAN_MOTOR_NAME);
  webots::Motor *tiltMotor = robot.getMotor(cfg::TILT_MOTOR_NAME);
  webots::PositionSensor *panSensor =
      panMotor ? panMotor->getPositionSensor() : nullptr;
  webots::PositionSensor *tiltSensor =
      tiltMotor ? tiltMotor->getPositionSensor() : nullptr;

  if (imu)
    imu->enable(ts);
  if (gyro)
    gyro->enable(ts);
  if (accel)
    accel->enable(ts);
  if (gps)
    gps->enable(ts);
  if (tfLuna)
    tfLuna->enable(ts);
  if (panSensor)
    panSensor->enable(ts);
  if (tiltSensor)
    tiltSensor->enable(ts);

  const bool tfDevicesReady =
      tfLuna && panMotor && tiltMotor && panSensor && tiltSensor;
  if (tfDevicesReady) {
    panMotor->setVelocity(cfg::TF_PAN_SPEED);
    panMotor->setPosition(cfg::TF_PAN_MAX);
    tiltMotor->setVelocity(1.0);
    tiltMotor->setPosition(0.0); // horizontal scan for 2-D obstacle mapping
  }
  if (!imu)
    std::printf("[init] WARNING: IMU '%s' missing\n", cfg::IMU_NAME);
  if (!gps)
    std::printf("[init] no GPS: position comes from IMU + wheel odometry only "
                "(will drift)\n");

  // ---- estimation ----
  EKF ekf;
  LowPass gyroZLP(8.0, dt);
  LowPass accelXLP(3.0, dt);
  TfLunaScanner tfScanner(cfg::TF_MIN_RANGE, cfg::TF_MAX_RANGE,
                          cfg::ROBOT_RADIUS + 0.15);
  ScanSummary scan;
  double lastTfSampleT = -1e9;
  double lastTfSamplePan = 1e9;
  double lastCompletedSweepT = -1e9;
  double panTarget = cfg::TF_PAN_MAX;
  bool haveTfSample = false;
  // ---- control ----
  PID speedPID(1.2, 0.4, 0.0, -1.0, 1.0, 0.05, 0.6);
  PID headingPID(1.0, 0.0, 0.05, -cfg::MAX_YAW_RATE, cfg::MAX_YAW_RATE);
  PID ratePID(0.5, 0.3, 0.0, -1.0, 1.0, 0.05, 0.4);
  RateLimiter vLimiter(cfg::MAX_ACCEL, cfg::MAX_DECEL);
  RateLimiter wLimiter(3.0, 3.0);

  // ---- planning ----
  OccupancyGrid grid(cfg::MAP_SIZE, cfg::MAP_RES);
  AStar astar;
  PlanResult plan;
  size_t wpIdx = 0;
  double lastPlanT = -1e9;
  if (cfg::USE_WATER_HINTS)
    for (const auto &h : cfg::WATER_HINTS)
      cfg::WATER.push_back(h);
  std::vector<bool> visited(cfg::WATER.size(), false);
  int goalIdx = -1;
  Vec2 goalPt;
  // RoverState state = RoverState::PLANNING;
  RoverState state = RoverState::INITIALIZING;

  int replanAttempts = 0;
  constexpr int MAX_REPLAN_ATTEMPTS = 3;

  int avoidanceTimeoutCount = 0;
  constexpr int MAX_AVOIDANCE_TIMEOUTS = 3;

  constexpr double SENSOR_WAIT_TIMEOUT_S = 5.0;
  double stateT = 0.0, lastVirtT = -1e9, lastLog = 0.0;
  double turnPickT = -1e9, turnHeading = 0.0;
  bool spinInit = false;
  double spinDone = 0.0, spinPrevYaw = 0.0;
  bool haveExpGoal = false, expPathOk = false;
  int expFailCount = 0; // FIX-OSC: consecutive EXPLORING plan failures
  Vec2 expGoal{0.0, 0.0};
  double expGoalT = 0.0;
  std::vector<Vec2> expBlacklist;
  std::vector<double> expBlacklistT;
  auto blacklistAdd = [&](const Vec2 &c) {
    const double t = robot.getTime();
    expBlacklist.push_back(c);
    expBlacklistT.push_back(t);
    for (int k = 0; k < 8; ++k) {
      const double a = k * PI / 4.0;
      expBlacklist.push_back(
          {c.x + 2.5 * std::cos(a), c.y + 2.5 * std::sin(a)});
      expBlacklistT.push_back(t);
    }
  };
  auto blacklistPrune = [&](double maxAge) {
    if (expBlacklistT.size() > expBlacklist.size())
      expBlacklistT.resize(expBlacklist.size());
    const double t = robot.getTime();
    std::vector<Vec2> keepV;
    std::vector<double> keepT;
    for (size_t i = 0; i < expBlacklist.size() && i < expBlacklistT.size(); ++i)
      if (t - expBlacklistT[i] < maxAge) {
        keepV.push_back(expBlacklist[i]);
        keepT.push_back(expBlacklistT[i]);
      }
    expBlacklist = keepV;
    expBlacklistT = keepT;
  };
  Vec2 home{cfg::START_X, cfg::START_Y};
  bool goingHome = false;
  int inspected = 0;
  std::vector<Vec2> inspectedAt;
  auto nearInspected = [&](const Vec2 &q) {
    for (const auto &a : inspectedAt)
      if (dist(a, q) < cfg::INSPECTED_EXCLUDE)
        return true;
    return false;
  };
  double spinStartT = 0.0;
  bool spinReported = false;
  double stallT = 0.0, recoverUntil = -1.0, recoverV = 0.0;
  double recoverW = 0.0;
  double rotStallT = 0.0; // time spent commanding a turn without turning
  double escapeUntil = -1.0, escapeHeading = 0.0;
  int homeFailCycles = 0;
  int errorCount = 0;
  int exploreSweeps = 0;
  //  bool missionIncomplete = false;
  if (robot.step(ts) == -1)
    return 0;
  {
    double yaw0 = imu ? imu->getRollPitchYaw()[2] : 0.0;
    double x0 = cfg::START_X, y0 = cfg::START_Y;
    if (gps) {
      const double *p = gps->getValues();
      x0 = p[0];
      y0 = p[1];
    }
    ekf.init(x0, y0, yaw0);
    home = {x0, y0};
  }
  double vCmdPrev = 0.0;

  auto setState = [&](RoverState s) {
    state = s;
    stateT = robot.getTime();
    speedPID.reset();
    headingPID.reset();
    ratePID.reset();
  };

  auto startReturn = [&]() {
    goingHome = true;
    goalIdx = -1;
    haveExpGoal = false;
    expPathOk = false;
    expFailCount = 0; // FIX-OSC
    lastPlanT = -1e9;
    for (size_t i = 0; i < cfg::WATER.size(); ++i)
      if (visited[i])
        grid.addVirtualObstacle(
            {cfg::WATER[i].x, cfg::WATER[i].y},
            std::max(cfg::POND_AVOID_RADIUS, cfg::WATER[i].radius + 1.0),
            robot.getTime() + 1e9);
    std::printf("[nav] returning to start (%.1f, %.1f)\n", home.x, home.y);
    setState(RoverState::PLANNING);
  };

  while (robot.step(ts) != -1) {
    const double now = robot.getTime();
    // INITIALIZING must always lead to WAITING_FOR_SENSORS first.
    if (state == RoverState::INITIALIZING) {
      std::printf("[init] starting sensor checks\n");
      setState(RoverState::WAITING_FOR_SENSORS);
    }
    // ============================ 1. sensing + filtering + EKF =============
    double yawRateRaw = 0.0, axRaw = 0.0, pitch = 0.0, imuYaw = ekf.yaw();
    if (imu) {
      const double *rpy = imu->getRollPitchYaw();
      pitch = rpy[1];
      imuYaw = rpy[2];
    }
    if (gyro)
      yawRateRaw = gyro->getValues()[2];
    if (accel)
      axRaw = accel->getValues()[0];

    const double G = 9.81;
    double axLin = axRaw + G * std::sin(pitch);

    double yawRate = gyroZLP.update(yawRateRaw);
    double aFwd = accelXLP.update(axLin);
    if (std::fabs(aFwd) < 0.03)
      aFwd = 0.0;

    ekf.predict(yawRate, aFwd, dt);
    if (imu)
      ekf.updateYaw(imuYaw);

    {
      double vObs = vCmdPrev;
      if (gps) {
        double s = gps->getSpeed();
        if (std::isfinite(s))
          vObs = (vCmdPrev < 0.0) ? -s : s;
      }
      ekf.updateSpeed(vObs);
    }
    if (gps) {
      const double *p = gps->getValues();
      ekf.updateGps(p[0], p[1]);
    }

    const Vec2 pos{ekf.x(), ekf.y()};
    const double yaw = ekf.yaw();
    const double vMeas = ekf.v();
    const double wMeas = yawRate;

    // Steep-slope guard: the rover cannot climb this, so back off and
    // mark the uphill direction as blocked for the planner.
    static double lastSlopeT = -1e9;
    if (std::fabs(pitch) > 0.35 && now - lastSlopeT > 3.0) {
      lastSlopeT = now;
      grid.addVirtualObstacle(
          {pos.x + 1.5 * std::cos(yaw), pos.y + 1.5 * std::sin(yaw)}, 1.2,
          now + 300.0);
      recoverUntil = now + cfg::RECOVER_TIME_S;
      recoverV = -cfg::RECOVER_SPEED;
      lastPlanT = -1e9;
      std::printf("[slope] tilt %.0f deg -> backing off, uphill blocked\n",
                  pitch * 180.0 / PI);
    }

    // ============================ 2. TF-Luna pan scan ======================
    if (tfDevicesReady) {
      const double measuredPan = panSensor->getValue();
      const double measuredTilt = tiltSensor->getValue();
      const double rawRange = tfLuna->getValue();

      // Sample at a fixed cadence and only after meaningful pan movement.
      if (now - lastTfSampleT >= cfg::TF_SAMPLE_PERIOD &&
          (!haveTfSample || std::fabs(measuredPan - lastTfSamplePan) >=
                                cfg::TF_SAMPLE_ANGLE_STEP)) {
        const double logicalTilt = -measuredTilt;
        if (tfScanner.addMeasurement(rawRange, measuredPan, logicalTilt, now)) {
          // Map this ray immediately using the rover pose at measurement time.
          // This avoids warping a moving sweep by applying only its final pose.
          const double c = std::cos(yaw), s = std::sin(yaw);
          const double originX =
              pos.x + cfg::TF_SENSOR_X * c - cfg::TF_SENSOR_Y * s;
          const double originY =
              pos.y + cfg::TF_SENSOR_X * s + cfg::TF_SENSOR_Y * c;
          const double worldBearing = yaw + measuredPan;
          const double horizontalRange = rawRange * std::cos(logicalTilt);
          const Vec2 endpoint{
              originX + horizontalRange * std::cos(worldBearing),
              originY + horizontalRange * std::sin(worldBearing)};
          const bool hit = rawRange < (cfg::TF_MAX_RANGE - 0.15);
          grid.integrateRay({originX, originY}, endpoint, hit);
        }
        lastTfSampleT = now;
        lastTfSamplePan = measuredPan;
        haveTfSample = true;
      }

      // Finish one one-way sweep at the commanded endpoint. Summarize the
      // completed sweep before clearing it and reversing the pan direction.
      const bool atEndpoint = std::fabs(measuredPan - panTarget) < 0.045;
      if (atEndpoint) {
        scan = tfScanner.summarize();
        lastCompletedSweepT = now;

        // Rays were integrated at acquisition time with the contemporaneous
        // EKF pose; do not re-project the completed sweep here.

        std::printf("[TF-Luna] sweep complete: %zu rays, front %.2fm left "
                    "%.2fm right %.2fm valid=%d\n",
                    scan.validMeasurements, scan.front, scan.left, scan.right,
                    scan.valid ? 1 : 0);
        tfScanner.beginScan();
        haveTfSample = false;
        lastTfSamplePan = 1e9;
        panTarget = (panTarget > 0.0) ? cfg::TF_PAN_MIN : cfg::TF_PAN_MAX;
        panMotor->setPosition(panTarget);
      }

      // Never allow an old completed sweep to authorize motion.
      if (lastCompletedSweepT < 0.0 ||
          now - lastCompletedSweepT > cfg::TF_SCAN_STALE_S)
        scan.valid = false;
    }

    // ============================ SENSOR READINESS =========================
    if (state == RoverState::WAITING_FOR_SENSORS) {
      if (imu && tfDevicesReady && scan.valid) {
        std::printf(
            "[init] IMU and TF-Luna pan scanner ready; entering PLANNING\n");
        lastPlanT = -1e9;
        setState(RoverState::PLANNING);
      } else if (now - stateT >= SENSOR_WAIT_TIMEOUT_S) {
        std::printf("[ERROR] sensor initialization timed out: IMU=%s "
                    "TF-Luna=%s scan=%s\n",
                    imu ? "OK" : "MISSING", tfDevicesReady ? "OK" : "MISSING",
                    scan.valid ? "OK" : "INVALID");
        setState(RoverState::ERROR);
      }
    }

    // ============================ 4. goal selection ========================
    if (state != RoverState::INITIALIZING &&
        state != RoverState::WAITING_FOR_SENSORS &&
        state != RoverState::REPLANNING &&
        state != RoverState::AVOIDING_OBSTACLE &&
        state != RoverState::ARRIVED && state != RoverState::DONE &&
        state != RoverState::ERROR && goalIdx < 0 && !goingHome) {
      double best = 1e18;
      for (size_t i = 0; i < cfg::WATER.size(); ++i) {
        if (visited[i] || nearInspected(Vec2{cfg::WATER[i].x, cfg::WATER[i].y}))
          continue;
        double d = dist(pos, {cfg::WATER[i].x, cfg::WATER[i].y});
        if (d < best) {
          best = d;
          goalIdx = static_cast<int>(i);
        }
      }
      if (goalIdx < 0) {
        if (state != RoverState::EXPLORING) {
          setState(RoverState::EXPLORING);
          std::printf("[nav] exploring for water\n");
        }
      } else {
        lastPlanT = -1e9;
        setState(RoverState::PLANNING);
      }
    }
    if (goingHome)
      goalPt = home;
    if (goalIdx >= 0) {
      const auto &w = cfg::WATER[goalIdx];
      double ang = std::atan2(pos.y - w.y, pos.x - w.x);
      double rr = w.radius + cfg::STANDOFF;
      goalPt = {w.x + rr * std::cos(ang), w.y + rr * std::sin(ang)};
    }

    // ============================ 5. behaviour / state machine =============
    double vTarget = 0.0, headingTarget = yaw;
    bool rotateInPlace = false;
    const bool escaping =
        now < escapeUntil && scan.valid && scan.front > cfg::STOP_DIST + 0.3 &&
        (state == RoverState::NAVIGATING || state == RoverState::EXPLORING ||
         state == RoverState::PLANNING || state == RoverState::REPLANNING);
    if (escaping) {
      headingTarget = escapeHeading;
      vTarget = cfg::ESCAPE_SPEED;
    } else
      switch (state) {
      case RoverState::REPLANNING: {
        // Never spin indefinitely while searching for another route.
        vTarget = 0.0;
        headingTarget = yaw;

        // If there is no destination to replan toward, return to exploration.
        if (goalIdx < 0 && !goingHome) {
          replanAttempts = 0;
          setState(RoverState::EXPLORING);
          break;
        }

        if (now - lastPlanT < cfg::REPLAN_PERIOD)
          break;

        lastPlanT = now;

        auto cost = grid.buildCostMap(now, cfg::ROBOT_RADIUS,
                                      cfg::INFLATION_RADIUS, 3.0);

        PlanResult p = astar.plan(grid, std::move(cost), pos, goalPt, 6);

        if (p.found && p.path.size() >= 2) {
          plan = std::move(p);
          wpIdx = 1;
          replanAttempts = 0;
          homeFailCycles = 0;
          errorCount = 0;
          expPathOk = true;
          haveExpGoal = false;

          std::printf("[nav] alternate route found\n");
          if (goingHome || goalIdx >= 0) {
            setState(RoverState::NAVIGATING);
          } else {
            setState(RoverState::EXPLORING);
          }
          break;
        }

        ++replanAttempts;

        std::printf("[nav] replan attempt %d/%d failed\n", replanAttempts,
                    MAX_REPLAN_ATTEMPTS);

        if (replanAttempts < MAX_REPLAN_ATTEMPTS)
          break;

        replanAttempts = 0;

        if (goingHome) {
          if (++homeFailCycles >= 4) {
            std::printf(
                "[ERROR] unable to find a route home; stopping rover\n");
            setState(RoverState::ERROR);
          } else {
            setState(RoverState::AVOIDING_OBSTACLE);
          }

        } else if (goalIdx >= 0) {
          std::printf("[nav] water target %d unreachable; abandoning target\n",
                      goalIdx);

          visited[goalIdx] = true;
          goalIdx = -1;
          haveExpGoal = false;
          expPathOk = false;
          expFailCount = 0;

          lastPlanT = -1e9;
          setState(RoverState::EXPLORING);
        } else {
          setState(RoverState::EXPLORING);
        }

        break;
      }
      case RoverState::PLANNING:
      case RoverState::NAVIGATING: {
        if (now - lastPlanT > cfg::REPLAN_PERIOD) {
          lastPlanT = now;
          auto cost = grid.buildCostMap(now, cfg::ROBOT_RADIUS,
                                        cfg::INFLATION_RADIUS, 3.0);
          PlanResult p = astar.plan(grid, std::move(cost), pos, goalPt, 6);
          if (p.found && p.path.size() >= 2) {
            plan = std::move(p);
            wpIdx = 1;
            if (state == RoverState::PLANNING)
              setState(RoverState::NAVIGATING);
          } else if (state == RoverState::NAVIGATING) {
            std::printf(
                "[nav] replanning failed, switching to bounded recovery\n");
            replanAttempts = 0;
            lastPlanT = -1e9;
            setState(RoverState::REPLANNING);
            break;
          } else if (state == RoverState::PLANNING) {
            std::printf(
                "[nav] initial plan failed; trying bounded replanning\n");
            replanAttempts = 0;
            lastPlanT = -1e9;
            setState(RoverState::REPLANNING);
            break;
          }
        }
        if (goingHome && dist(pos, home) < cfg::HOME_TOL) {
          if (inspected >= cfg::NUM_WATER_BODIES) {
            std::printf("[nav] back at start (%.2f, %.2f) - mission complete "
                        "(%d/%d samples)\n",
                        pos.x, pos.y, inspected, cfg::NUM_WATER_BODIES);
            setState(RoverState::DONE);
            break;
          }
          std::printf("[nav] at start with only %d/%d samples - resuming "
                      "search\n",
                      inspected, cfg::NUM_WATER_BODIES);
          goingHome = false;
          goalIdx = -1;
          haveExpGoal = false;
          expPathOk = false;
          expBlacklist.clear();
          spinInit = false;
          spinDone = 0.0;
          spinReported = false;
          lastPlanT = -1e9;
          setState(RoverState::EXPLORING);
          break;
        }
        double shoreDist = 1e9;
        if (!goingHome) {
          const auto &w = cfg::WATER[goalIdx];
          shoreDist = dist(pos, {w.x, w.y}) - w.radius;
        }
        if (!goingHome && (dist(pos, goalPt) < cfg::ARRIVE_TOL ||
                           shoreDist < cfg::STANDOFF + 0.3)) {
          std::printf("[nav] reached water body %d (shore distance %.2f m)\n",
                      goalIdx, shoreDist);
          visited[goalIdx] = true;
          {
            auto &wr = cfg::WATER[goalIdx];
            double ux = wr.x - pos.x, uy = wr.y - pos.y;
            double un = std::hypot(ux, uy);
            if (un > 1e-3 && wr.radius <= 0.0) {
              wr.x += ux / un * cfg::POND_RADIUS_GUESS;
              wr.y += uy / un * cfg::POND_RADIUS_GUESS;
            }
            for (size_t k = 0; k < cfg::WATER.size(); ++k)
              if (!visited[k] &&
                  dist(Vec2{cfg::WATER[k].x, cfg::WATER[k].y},
                       Vec2{wr.x, wr.y}) < cfg::SAME_POND_VISITED)
                visited[k] = true;
          }
          setState(RoverState::ARRIVED);
          break;
        }

        if (now < escapeUntil && scan.valid &&
            scan.front > cfg::STOP_DIST + 0.4) {
          headingTarget = escapeHeading;
          vTarget = cfg::ESCAPE_SPEED;
          break;
        }

        // FIX-OSC: guard against a stale plan whose wpIdx has run off the end.
        if (plan.path.empty() || wpIdx >= plan.path.size()) {
          expPathOk = false;
          headingTarget = yaw;
          vTarget = 0.15;
          break;
        }

        while (wpIdx + 1 < plan.path.size() &&
               dist(pos, plan.path[wpIdx]) < 0.6)
          ++wpIdx;
        Vec2 target = plan.path[wpIdx];
        for (size_t j = wpIdx; j < plan.path.size(); ++j) {
          if (dist(pos, plan.path[j]) <= cfg::LOOKAHEAD)
            target = plan.path[j];
          else
            break;
        }
        // If the lookahead target is behind us, turn toward it in place.
        {
          double dx = target.x - pos.x, dy = target.y - pos.y;
          if (dx * std::cos(yaw) + dy * std::sin(yaw) < 0.0) {
            headingTarget = std::atan2(dy, dx);
            rotateInPlace = true;
            vTarget = 0.0;
            break;
          }
        }
        headingTarget = std::atan2(target.y - pos.y, target.x - pos.x);
        double err = wrapAngle(headingTarget - yaw);

        double v =
            cfg::MAX_SPEED * clamp(1.0 - std::fabs(err) / (PI / 2.0), 0.0, 1.0);
        v = std::min(v, 0.25 + 0.4 * dist(pos, goalPt));
        if (scan.valid) {
          if (scan.front < cfg::STOP_DIST)
            v = 0.0;
          else if (scan.front < cfg::SLOW_DIST)
            v *= (scan.front - cfg::STOP_DIST) /
                 (cfg::SLOW_DIST - cfg::STOP_DIST);
          if (scan.front < cfg::SLOW_DIST) {
            double nudge = clamp((scan.right - scan.left) / 4.0, -0.5, 0.5);
            headingTarget = wrapAngle(headingTarget + nudge * 0.6);
          }
        }
        vTarget = v;

        if (scan.valid && scan.front < cfg::STOP_DIST)
          setState(RoverState::AVOIDING_OBSTACLE);
        break;
      }

      case RoverState::EXPLORING: {
        // 0) one 360-degree look-around at startup
        if (!spinInit) {
          spinInit = true;
          spinPrevYaw = yaw;
          spinStartT = now;
        }
        bool spinStuck = (now - spinStartT > 6.0 && spinDone < 0.35);
        if (spinDone < PI && now - spinStartT < cfg::SPIN_TIMEOUT_S &&
            !spinStuck) {
          spinDone += std::fabs(wrapAngle(yaw - spinPrevYaw));
          spinPrevYaw = yaw;
          rotateInPlace = true;
          headingTarget = yaw + 0.8;
          break;
        }
        if (!spinReported) {
          spinReported = true;
          std::printf("[nav] look-around over: turned %.0f deg in %.1f s\n",
                      spinDone * 180.0 / PI, now - spinStartT);
        }
        // 1) water too close: turn away
        // 2) pick a frontier goal
        if (!haveExpGoal || dist(pos, expGoal) < 1.0 || now - expGoalT > 25.0) {
          if (haveExpGoal && dist(pos, expGoal) >= 1.0)
            blacklistAdd(expGoal);
          haveExpGoal = false;
          Vec2 g;
          if (grid.nearestFrontier(pos, cfg::ARENA_HALF, 3.0, expBlacklist,
                                   g)) {
            expGoal = g;
            haveExpGoal = true;
            expPathOk = false;
            expFailCount = 0;
            expGoalT = now;
            lastPlanT = -1e9;
            std::printf("[nav] exploring toward (%.1f,%.1f)\n", g.x, g.y);
          } else {
            std::printf("[nav] nothing left to explore\n");
            if (inspected < cfg::NUM_WATER_BODIES) {
              ++exploreSweeps;
              std::printf("[nav] only %d of %d samples done; searching again "
                          "(sweep %d)\n",
                          inspected, cfg::NUM_WATER_BODIES, exploreSweeps);
              blacklistPrune(90.0);
              spinInit = false;
              spinDone = 0.0;
              spinReported = false;
              lastPlanT = -1e9;
              if (scan.valid && scan.front < cfg::CLEAR_DIST)
                setState(RoverState::AVOIDING_OBSTACLE); // back out first
              break;
            }
            startReturn();
            break;
          }
        }
        // 3) plan about once per second
        if (now - lastPlanT > cfg::REPLAN_PERIOD) {
          lastPlanT = now;
          auto cost = grid.buildCostMap(now, cfg::ROBOT_RADIUS,
                                        cfg::INFLATION_RADIUS, 3.0);
          PlanResult p = astar.plan(grid, std::move(cost), pos, expGoal, 6);
          if (p.found && p.path.size() >= 2) {
            plan = std::move(p);
            wpIdx = 1;
            expPathOk = true;
            expFailCount = 0;
          } else {
            // FIX-OSC: do not use a stale plan (its last waypoint may be
            // behind us) and do not blacklist on the first failure
            // (that flips the frontier east-west-east-west every second).
            // Drive straight for this cycle, count failures, blacklist at 4.
            expPathOk = false;
            if (++expFailCount >= 3) {
              blacklistAdd(expGoal);
              haveExpGoal = false;
              expFailCount = 0;
            }
            std::printf("[explore] plan fail #%d (goal %.1f,%.1f)\n",
                        expFailCount, expGoal.x, expGoal.y);
            // A failed plan is not permission to drive blindly.
            if (expFailCount >= 2 && scan.valid &&
                scan.front < cfg::CLEAR_DIST) {
              setState(RoverState::AVOIDING_OBSTACLE);
              break;
            }
            headingTarget = yaw;
            vTarget = 0.0;
            break;
          }
        }
        if (!expPathOk)
          break;

        // The route is exhausted. Stop and select a different frontier.
        if (plan.path.empty() || (wpIdx >= plan.path.size() - 1 &&
                                  dist(pos, plan.path.back()) < 0.6)) {
          expPathOk = false;
          vTarget = 0.0;

          if (haveExpGoal && dist(pos, expGoal) >= 1.0) {
            blacklistAdd(expGoal);
          }

          haveExpGoal = false;
          expFailCount = 0;
          lastPlanT = -1e9;
          break;
        }

        // 4) pure pursuit
        while (wpIdx + 1 < plan.path.size() &&
               dist(pos, plan.path[wpIdx]) < 0.6)
          ++wpIdx;
        Vec2 target = plan.path[wpIdx];
        for (size_t j = wpIdx; j < plan.path.size(); ++j) {
          if (dist(pos, plan.path[j]) <= cfg::LOOKAHEAD)
            target = plan.path[j];
          else
            break;
        }
        // If the waypoint is behind us, turn toward it in place.
        {
          double dx = target.x - pos.x, dy = target.y - pos.y;
          if (dx * std::cos(yaw) + dy * std::sin(yaw) < 0.0) {
            headingTarget = std::atan2(dy, dx);
            rotateInPlace = true;
            vTarget = 0.0;
            break;
          }
        }
        headingTarget = std::atan2(target.y - pos.y, target.x - pos.x);
        double err = wrapAngle(headingTarget - yaw);
        double v =
            cfg::MAX_SPEED * clamp(1.0 - std::fabs(err) / (PI / 2.0), 0.0, 1.0);
        v = std::min(v, 0.6);
        if (scan.valid) {
          if (scan.front < cfg::STOP_DIST)
            v = 0.0;
          else if (scan.front < cfg::SLOW_DIST)
            v *= (scan.front - cfg::STOP_DIST) /
                 (cfg::SLOW_DIST - cfg::STOP_DIST);
        }
        vTarget = v;
        if (scan.valid && scan.front < cfg::STOP_DIST)
          setState(RoverState::AVOIDING_OBSTACLE);
        break;
      }

      case RoverState::AVOIDING_OBSTACLE: {
        // Never move forward while the obstacle is too close.
        vTarget = 0.0;

        // Missing/invalid TF-Luna scan is not evidence of a clear path.
        if (!scan.valid) {
          headingTarget = yaw;

          if (now - stateT > 2.0) {
            std::printf(
                "[ERROR] TF-Luna scan invalid during obstacle avoidance; "
                "stopping rover\n");
            setState(RoverState::ERROR);
          }

          break;
        }

        // The path is clear: leave avoidance and replan from current position.
        if (scan.front > cfg::CLEAR_DIST) {
          escapeHeading = yaw;
          escapeUntil = now + cfg::ESCAPE_TIME_S;
          avoidanceTimeoutCount = 0;
          expPathOk = false;
          expFailCount = 0;
          lastPlanT = -1e9;

          std::printf("[avoid] path clear; replanning\n");

          setState((goalIdx >= 0 || goingHome) ? RoverState::PLANNING
                                               : RoverState::EXPLORING);

          break;
        }

        // Keep the rover stationary while deciding which way to turn.
        // Turn toward the side with more free space.
        rotateInPlace = true;

        double turnDirection = (scan.left >= scan.right) ? 1.0 : -1.0;

        headingTarget = wrapAngle(yaw + turnDirection * 0.7);

        // Do not keep turning indefinitely.
        if (now - stateT > 4.0) {
          if (scan.front < 1.2) {                     // <-- new
            recoverUntil = now + cfg::RECOVER_TIME_S; // <-- new
            recoverV = -cfg::RECOVER_SPEED;           // <-- new
          }
          ++avoidanceTimeoutCount;

          std::printf(
              "[avoid] attempt %d/%d timed out; requesting replanning\n",
              avoidanceTimeoutCount, MAX_AVOIDANCE_TIMEOUTS);

          lastPlanT = -1e9;
          expPathOk = false;
          expFailCount = 0;

          if (avoidanceTimeoutCount >= MAX_AVOIDANCE_TIMEOUTS) {
            std::printf("[ERROR] repeated obstacle-avoidance failures; "
                        "stopping rover\n");

            setState(RoverState::ERROR);
          } else {
            setState(RoverState::REPLANNING);
          }
        }

        break;
      }
      case RoverState::ARRIVED: {
        vTarget = 0.0;
        headingTarget = yaw;
        if (now - stateT > cfg::WAIT_AT_WATER_S) {
          goalIdx = -1;
          haveExpGoal = false;
          expPathOk = false;
          expFailCount = 0;
          ++inspected;
          inspectedAt.push_back(pos);
          exploreSweeps = 0; // <-- add this line
          std::printf("[nav] inspected %d of %d water bodies\n", inspected,
                      cfg::NUM_WATER_BODIES);
          if (inspected >= cfg::NUM_WATER_BODIES)
            startReturn();
          else
            setState(RoverState::EXPLORING);
        }
        break;
      }

      case RoverState::ERROR: {
        vTarget = 0.0;
        headingTarget = yaw;
        if (!imu || !tfDevicesReady)
          break;
        const double waitS = std::min(3.0 + 2.0 * errorCount, 15.0);
        if (now - stateT < waitS)
          break;
        ++errorCount;
        std::printf("[recover] leaving ERROR (attempt %d)\n", errorCount);
        expBlacklist.clear();
        haveExpGoal = false;
        expPathOk = false;
        expFailCount = 0;
        replanAttempts = 0;
        homeFailCycles = 0;
        avoidanceTimeoutCount = 0;
        lastPlanT = -1e9;
        if (!scan.valid) {
          setState(RoverState::WAITING_FOR_SENSORS);
        } else {
          if (scan.front < 1.5) {
            recoverUntil = now + 2.0 * cfg::RECOVER_TIME_S;
            recoverV = -cfg::RECOVER_SPEED;
          }
          setState(RoverState::AVOIDING_OBSTACLE);
        }
        break;
      }

      case RoverState::DONE:
      default:
        vTarget = 0.0;
        headingTarget = yaw;
        break;
      }

    // ============================ 6. PID loops =============================
    double vCmd = vTarget + speedPID.update(vTarget - vMeas, dt);
    vCmd = clamp(vCmd, 0.0, cfg::MAX_SPEED);
    if (vTarget <= 0.0 && std::fabs(vMeas) < 0.05) {
      vCmd = 0.0;
      speedPID.reset();
    }
    vCmd = vLimiter.update(vCmd, dt);
    vCmdPrev = vCmd;

    double headingErr = wrapAngle(headingTarget - yaw);
    double wTarget = headingPID.update(headingErr, dt);
    if (rotateInPlace)
      wTarget = clamp(wTarget, -0.8, 0.8);
    double wCmd = wTarget + ratePID.update(wTarget - wMeas, dt);
    // Rotating in place can stall on soft terrain: the wheels spin but the
    // rover does not turn. Detect that, raise the yaw command, and if it is
    // still stuck, drive a short unstick move.
    if (std::fabs(wTarget) > 0.3 && std::fabs(wMeas) < 0.08)
      rotStallT += dt;
    else
      rotStallT = 0.0;
    const double yawLimit =
        rotateInPlace ? cfg::MAX_YAW_RATE + std::min(0.7, 0.3 * rotStallT)
                      : cfg::MAX_YAW_RATE;
    wCmd = clamp(wCmd, -yawLimit, yawLimit);
    if (rotStallT > 2.0) {
      rotStallT = 0.0;
      recoverUntil = now + cfg::RECOVER_TIME_S;
      recoverV = (scan.valid && scan.front < 1.5) ? -cfg::RECOVER_SPEED
                                                  : cfg::RECOVER_SPEED;
      recoverW = (scan.valid && scan.right > scan.left) ? -0.6 : 0.6;
      std::printf("[warn] rotation stalled (gyro %.2f) -> unstick %s\n", wMeas,
                  recoverV < 0.0 ? "reverse" : "forward");
    }
    if (state == RoverState::DONE || state == RoverState::ARRIVED) {
      wCmd = 0.0;
      ratePID.reset();
    }
    wCmd = wLimiter.update(wCmd, dt);

    const double gpsSpd = (gps && std::isfinite(gps->getSpeed()))
                              ? gps->getSpeed()
                              : std::fabs(vMeas);
    const bool safeStopState = state == RoverState::DONE ||
                               state == RoverState::ARRIVED ||
                               state == RoverState::INITIALIZING ||
                               state == RoverState::WAITING_FOR_SENSORS;

    if (safeStopState) {
      // Cancel any ongoing stall recovery.
      recoverUntil = -1.0;
      stallT = 0.0;

      // Override all movement commands.
      vCmd = 0.0;
      wCmd = 0.0;

      // Prevent the rate limiters from retaining a movement command.
      speedPID.reset();
      headingPID.reset();
      ratePID.reset();
      vLimiter.reset(0.0);
      wLimiter.reset(0.0);
    } else if (now < recoverUntil) {
      vCmd = recoverV;
      wCmd = recoverW;
    } else {
      if (recoverUntil > 0.0) {
        recoverUntil = -1.0;
        speedPID.reset();
        headingPID.reset();
        ratePID.reset();
        vLimiter.reset(0.0);
        wLimiter.reset(0.0);
      }
      bool tryingToTranslate = !rotateInPlace && std::fabs(vCmd) > 0.08;
      bool moving = gpsSpd > 0.04;
      stallT = (tryingToTranslate && !moving) ? stallT + dt : 0.0;
      if (stallT > cfg::STALL_TIME_S) {
        stallT = 0.0;
        recoverUntil = now + cfg::RECOVER_TIME_S;
        recoverV = (scan.valid && scan.front < 1.5) ? -cfg::RECOVER_SPEED
                                                    : cfg::RECOVER_SPEED;
        std::printf(
            "[warn] stalled (cmd v=%.2f w=%.2f, gps %.2f m/s, gyro %.2f) "
            "-> unstick %s\n",
            vCmd, wCmd, gpsSpd, wMeas, recoverV < 0.0 ? "reverse" : "forward");
        vCmd = recoverV;
        wCmd = 0.0;
      }
    }
    vCmdPrev = vCmd;

    // ============================ 7. skid-steer mixing =====================
    WheelSpeeds ws =
        mixSkidSteer(vCmd, wCmd, cfg::TRACK_WIDTH * cfg::SLIP_FACTOR,
                     cfg::WHEEL_RADIUS, maxWheelRadS);
    for (auto *m : leftM)
      m->setVelocity(cfg::LEFT_DIR * ws.leftRadS);
    for (auto *m : rightM)
      m->setVelocity(cfg::RIGHT_DIR * ws.rightRadS);

    // ============================ 8. logging ===============================
    if (now - lastLog > 1.0) {
      lastLog = now;
      std::printf("[%6.1fs] %-19s pos(%.2f,%.2f) yaw %.0fdeg v %.2f/%.2f w "
                  "%.2f/%.2f TF(front %.1f left %.1f right %.1f valid %d) "
                  "goal %d wheels(%.2f %.2f rad/s) gps %.2f\n",
                  now, stateName(state), pos.x, pos.y, yaw * 180.0 / PI, vMeas,
                  vCmd, wMeas, wCmd, scan.front, scan.left, scan.right,
                  scan.valid ? 1 : 0, goalIdx, ws.leftRadS, ws.rightRadS,
                  gpsSpd);
      std::fflush(stdout);
    }
  }
  return 0;
}
