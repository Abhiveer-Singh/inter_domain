#ifndef ROVER_STATE_MACHINE_HPP
#define ROVER_STATE_MACHINE_HPP

namespace rover {

// The controller's single source of truth for "what is the rover doing".
//   INITIALIZING        : start-up, immediately moves on to WAITING_FOR_SENSORS
//   WAITING_FOR_SENSORS : waits for IMU + lidar scan (times out to ERROR)
//   CALIBRATING         : runs the SensorCalibrator self-test (gyro bias,
//                         accel/tilt, GPS, lidar, camera, encoders, drive and
//                         turn tests); on pass -> PLANNING, on fail -> ERROR
//   PLANNING            : A* is (re)computing a path to the current goal
//   NAVIGATING          : following the A* path (pure pursuit)
//   REPLANNING          : bounded retries after a planning failure
//   EXPLORING           : no water known yet: look around, then drive to map
//                         frontiers
//   AVOIDING_OBSTACLE   : lidar says blocked ahead: stop and turn to open space
//   ARRIVED             : at a water body, pausing to inspect it
//   DONE                : mission finished (back at start)
//   ERROR               : stopped; waits, then tries to recover
enum class RoverState {
  INITIALIZING,
  WAITING_FOR_SENSORS,
  CALIBRATING,
  PLANNING,
  NAVIGATING,
  REPLANNING,
  EXPLORING,
  AVOIDING_OBSTACLE,
  ARRIVED,
  DONE,
  ERROR
};

inline const char *stateName(RoverState state) {
  switch (state) {
  case RoverState::INITIALIZING:
    return "INITIALIZING";

  case RoverState::WAITING_FOR_SENSORS:
    return "WAITING_FOR_SENSORS";

  case RoverState::CALIBRATING:
    return "CALIBRATING";

  case RoverState::PLANNING:
    return "PLANNING";

  case RoverState::NAVIGATING:
    return "NAVIGATING";

  case RoverState::REPLANNING:
    return "REPLANNING";

  case RoverState::EXPLORING:
    return "EXPLORING";

  case RoverState::AVOIDING_OBSTACLE:
    return "AVOIDING_OBSTACLE";

  case RoverState::ARRIVED:
    return "ARRIVED";

  case RoverState::DONE:
    return "DONE";

  case RoverState::ERROR:
    return "ERROR";

  default:
    return "UNKNOWN";
  }
}

} // namespace rover

#endif // ROVER_STATE_MACHINE_HPP
