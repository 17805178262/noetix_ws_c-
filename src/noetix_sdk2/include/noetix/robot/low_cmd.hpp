#ifndef NOETIX_ROBOT_LOW_CMD_HPP
#define NOETIX_ROBOT_LOW_CMD_HPP

#include <cstdint>
#include <vector>

#include "noetix/robot/motor_cmd.hpp"

namespace noetix {

/**
 * Complete low-level command packet.
 *
 * There is deliberately no sparse-command or hold semantics in SDK2.  A
 * command handed to RobotInterface::write() contains one MotorCmd for every
 * configured actuator.  The bridge may reorder a complete named ROS packet
 * once before it reaches the SDK; numeric command values are not reshaped.
 */
struct LowCmd {
  std::uint64_t timestamp_ns{0};
  std::uint64_t sequence{0};
  std::vector<MotorCmd> motor_cmd;

  void resize(std::size_t n) { motor_cmd.resize(n); }
  std::size_t size() const noexcept { return motor_cmd.size(); }
  std::size_t dof() const noexcept { return motor_cmd.size(); }
  bool empty() const noexcept { return motor_cmd.empty(); }
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_LOW_CMD_HPP
