#ifndef NOETIX_CONFIG_ROBOT_CONFIG_HPP_
#define NOETIX_CONFIG_ROBOT_CONFIG_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace noetix {

struct RobotConfig {
  std::string robot_name;
  std::string config_root;
  std::size_t dof{0};

  // Fixed backend actuator order.  LowCmd presented to RobotInterface::write
  // must already contain exactly this full set in this order.
  std::vector<std::string> actuator_order;
};

}  // namespace noetix

#endif  // NOETIX_CONFIG_ROBOT_CONFIG_HPP_
