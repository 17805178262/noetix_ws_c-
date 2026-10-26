#ifndef NOETIX_HARDWARE_ACTUATOR_CMD_HPP_
#define NOETIX_HARDWARE_ACTUATOR_CMD_HPP_

#include <string>

namespace noetix {

/** Named actuator-space MIT command. */
struct ActuatorCmd {
  std::string name;
  float position{0.0F};
  float velocity{0.0F};
  float torque_ff{0.0F};
  float kp{0.0F};
  float kd{0.0F};
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_ACTUATOR_CMD_HPP_
