#ifndef NOETIX_ROBOT_MAPPED_ROBOT_HPP_
#define NOETIX_ROBOT_MAPPED_ROBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "noetix/hardware/hardware_backend.hpp"
#include "noetix/robot/robot_interface.hpp"

namespace noetix {

enum class HardwareKind {
  kN3,
  kBumi,
};

/** Thin RobotInterface adapter for N3 / Bumi real-hardware backends. */
class MappedRobot : public RobotInterface {
 public:
  explicit MappedRobot(HardwareKind hardware_kind);

  bool configure(const RobotConfig& config) override;
  bool start() override;
  bool stop() override;
  bool read(LowState& state) override;
  bool write(const LowCmd& command) override;
  bool enableActuators() override;
  bool disableActuators() override;
  bool actuatorsEnabled() const override;
  RobotStatus status() const override;

 private:
  bool createBackend();

  HardwareKind hardware_kind_;
  std::unique_ptr<HardwareBackend> backend_;
  std::vector<std::string> actuator_names_;
  std::vector<ActuatorState> actuator_state_;
  std::vector<ActuatorCmd> actuator_cmd_;
  RobotStatus status_;
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_MAPPED_ROBOT_HPP_
