#ifndef NOETIX_ROBOT_MOTOR_CMD_HPP
#define NOETIX_ROBOT_MOTOR_CMD_HPP

#include <string>

namespace noetix {

/**
 * @brief Named low-level actuator command.
 *
 * Contract:
 * - name is the actuator/motor name understood by the hardware driver.
 * - q/dq/tau are actuator-space values, not URDF joint-space values.
 * - LowCmd may be sparse at the ROS/API boundary. Robot implementations may
 *   expand a sparse command to a full command by holding unmentioned motors.
 */
struct MotorCmd {
  std::string name;
  float q{0.0F};
  float dq{0.0F};
  float tau{0.0F};
  float kp{0.0F};
  float kd{0.0F};
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_MOTOR_CMD_HPP
