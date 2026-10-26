#include "robot/mapped_robot.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

#if NOETIX_HAS_N3_HW
#include "n3/n3_backend.hpp"
#endif
#if NOETIX_HAS_BUMI_HW
#include "noetix/hardware/bumi/bumi_backend.hpp"
#endif

namespace noetix {

MappedRobot::MappedRobot(HardwareKind hardware_kind)
    : hardware_kind_(hardware_kind) {}

bool MappedRobot::createBackend() {
  if (hardware_kind_ == HardwareKind::kN3) {
#if NOETIX_HAS_N3_HW
    backend_ = std::make_unique<N3Backend>();
    return true;
#else
    std::cerr << "[MappedRobot] N3 real-hardware backend was not built: "
              << NOETIX_N3_HW_UNAVAILABLE_REASON << '\n'
              << "[MappedRobot] Install the matching N3 vendor archive and headers, "
                 "then rebuild noetix_sdk2 and noetix_bridge with "
                 "-DNOETIX_REQUIRE_N3_HW=ON."
              << std::endl;
    return false;
#endif
  }

#if NOETIX_HAS_BUMI_HW
  backend_ = std::make_unique<BumiBackend>();
  return true;
#else
  std::cerr << "[MappedRobot] Bumi real-hardware backend was not built." << std::endl;
  return false;
#endif
}

bool MappedRobot::configure(const RobotConfig& config) {
  if (config.dof == 0U || config.config_root.empty()) {
    std::cerr << "[MappedRobot] Invalid RobotConfig." << std::endl;
    return false;
  }
  if (config.actuator_order.size() != config.dof) {
    std::cerr << "[MappedRobot] actuator_order size must equal dof." << std::endl;
    return false;
  }

  actuator_names_ = config.actuator_order;
  for (std::size_t i = 0; i < actuator_names_.size(); ++i) {
    if (actuator_names_[i].empty()) {
      std::cerr << "[MappedRobot] empty actuator name at " << i << std::endl;
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (actuator_names_[j] == actuator_names_[i]) {
        std::cerr << "[MappedRobot] duplicate actuator name: "
                  << actuator_names_[i] << std::endl;
        return false;
      }
    }
  }

  if (!createBackend()) return false;

  actuator_state_.assign(config.dof, ActuatorState{});
  actuator_cmd_.assign(config.dof, ActuatorCmd{});
  for (std::size_t i = 0; i < config.dof; ++i) {
    actuator_cmd_[i].name = actuator_names_[i];
  }

  if (!backend_->configure(config)) return false;

  status_ = RobotStatus{};
  status_.mode = RobotMode::kPassive;
  return true;
}

bool MappedRobot::start() {
  if (!backend_) return false;
  const bool ok = backend_->start();
  status_.connected = ok;
  status_.enabled = ok && backend_->actuatorsEnabled();
  status_.mode = status_.enabled ? RobotMode::kControl : RobotMode::kPassive;
  return ok;
}

bool MappedRobot::stop() {
  if (!backend_) return false;
  const bool ok = backend_->stop();
  status_.connected = false;
  status_.enabled = false;
  status_.mode = RobotMode::kPassive;
  return ok;
}

bool MappedRobot::read(LowState& state) {
  if (!backend_ || actuator_state_.size() != actuator_names_.size()) return false;
  if (!backend_->read(actuator_state_)) {
    status_.connected = false;
    state.status = status_;
    return false;
  }

  status_.connected = true;
  status_.enabled = backend_->actuatorsEnabled();
  status_.mode = status_.enabled ? RobotMode::kControl : RobotMode::kPassive;

  if (state.motor_state.size() != actuator_state_.size()) {
    state.resize(actuator_state_.size());
  }

  for (std::size_t i = 0; i < actuator_state_.size(); ++i) {
    const auto& src = actuator_state_[i];
    auto& dst = state.motor_state[i];
    dst.name = actuator_names_[i];
    dst.q = src.position;
    dst.dq = src.velocity;
    dst.tau_est = src.torque;
    dst.temperature = src.temperature;
    dst.error = src.error;
  }

  ImuState imu;
  if (backend_->readImu(imu)) {
    state.imu = imu;
  } else {
    state.imu.valid = false;
  }

  state.status = status_;
  return true;
}

bool MappedRobot::write(const LowCmd& command) {
  if (!backend_ || command.motor_cmd.size() != actuator_cmd_.size()) {
    return false;
  }

  // The bridge already normalizes one complete command into actuator order.
  // Do not fill missing joints, hold positions, clamp values, or perform a
  // second name lookup here.  This layer is intentionally a direct copy.
  for (std::size_t i = 0; i < actuator_cmd_.size(); ++i) {
    const auto& src = command.motor_cmd[i];
    if (src.name != actuator_names_[i]) {
      std::cerr << "[MappedRobot] command order mismatch at " << i
                << ": expected=" << actuator_names_[i]
                << " got=" << src.name << std::endl;
      return false;
    }

    auto& dst = actuator_cmd_[i];
    dst.position = src.q;
    dst.velocity = src.dq;
    dst.torque_ff = src.tau;
    dst.kp = src.kp;
    dst.kd = src.kd;
  }

  return backend_->write(actuator_cmd_);
}

bool MappedRobot::enableActuators() {
  if (!backend_) return false;
  const bool ok = backend_->enableActuators();
  status_.enabled = backend_->actuatorsEnabled();
  status_.mode = status_.enabled ? RobotMode::kControl : RobotMode::kPassive;
  return ok && status_.enabled;
}

bool MappedRobot::disableActuators() {
  if (!backend_) return false;
  const bool ok = backend_->disableActuators();
  status_.enabled = backend_->actuatorsEnabled();
  status_.mode = status_.enabled ? RobotMode::kControl : RobotMode::kPassive;
  return ok && !status_.enabled;
}

bool MappedRobot::actuatorsEnabled() const {
  return backend_ && backend_->actuatorsEnabled();
}

RobotStatus MappedRobot::status() const {
  RobotStatus result = status_;
  result.enabled = actuatorsEnabled();
  return result;
}

}  // namespace noetix
