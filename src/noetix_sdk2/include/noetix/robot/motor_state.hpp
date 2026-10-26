#ifndef NOETIX_ROBOT_MOTOR_STATE_HPP
#define NOETIX_ROBOT_MOTOR_STATE_HPP

#include <cstdint>
#include <string>

namespace noetix {

/**
 * @brief Named low-level actuator state.
 *
 * q/dq/tau_est are actuator-space values. ROS joint-space conversion belongs
 * to an adapter layer (for example /joint_states generation), not the low-level
 * SDK data path.
 */
struct MotorState {
  std::string name;
  float q{0.0F};
  float dq{0.0F};
  float tau_est{0.0F};
  float temperature{0.0F};
  std::uint32_t error{0};
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_MOTOR_STATE_HPP
