#include "noetix/robot/robot_factory.hpp"

#include <stdexcept>

#include "robot/mapped_robot.hpp"

namespace noetix {

std::unique_ptr<RobotInterface> CreateRobot(const RobotConfig& config) {
  if (config.robot_name == "n3") {
    return std::make_unique<MappedRobot>(HardwareKind::kN3);
  }
  if (config.robot_name == "bumi3" || config.robot_name == "bumi4340" ||
      config.robot_name == "bumi2") {
    return std::make_unique<MappedRobot>(HardwareKind::kBumi);
  }
  throw std::invalid_argument("Unsupported robot_name: " + config.robot_name);
}

}  // namespace noetix
