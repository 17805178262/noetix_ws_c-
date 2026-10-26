#include "noetix/hardware/bumi/bumi_controller.hpp"

#include <iostream>
#include <cmath>

#include "common_type.h"
#include "hw_controller.h"

namespace noetix {

class BumiController::Impl {
 public:
  legged::HwController* hw{nullptr};

  static legged::CtrlChannel channel(int value) {
    switch (value) {
      case 1: return legged::CtrlChannel::CTRL_CH1;
      case 2: return legged::CtrlChannel::CTRL_CH2;
      case 3: return legged::CtrlChannel::CTRL_CH3;
      default: return legged::CtrlChannel::CTRL_CH1;
    }
  }

  static legged::ActuatorType type(const std::string& value) {
    if (value == "ACTR_4340") return legged::ActuatorType::ACTR_4340;
    if (value == "ACTR_4315") return legged::ActuatorType::ACTR_4315;
    if (value == "ACTR_4325") return legged::ActuatorType::ACTR_4325;
    if (value == "ACTR_RS05") return legged::ActuatorType::ACTR_RS05;
    if (value == "ACTR_4340_ANK") return legged::ActuatorType::ACTR_4340_ANK;
    if (value == "ACTR_4308") return legged::ActuatorType::ACTR_4308;
    if (value == "ACTR_F5014") return legged::ActuatorType::ACTR_F5014;
    if (value == "OMNI_PICKER") return legged::ActuatorType::OMNI_PICKER;
    return legged::ActuatorType::UNKOWN;
  }
};

BumiController::BumiController() : impl_(new Impl()) {
  impl_->hw = legged::HwController::GetInstance();
}

BumiController::~BumiController() {
  delete impl_;
  impl_ = nullptr;
}

bool BumiController::createCanu(const std::string& name, int ecat_id,
                                const std::string& robot_model) {
  if (!impl_ || !impl_->hw || ecat_id < 0 || ecat_id > 255) return false;
  return impl_->hw->CreateCanu(name, static_cast<std::uint8_t>(ecat_id), robot_model);
}

bool BumiController::attachActuator(const std::string& canu_name, int channel,
                                    const std::string& type,
                                    const std::string& actuator_name, int can_id) {
  if (!impl_ || !impl_->hw || channel < 1 || channel > 3 ||
      can_id < 0 || can_id > 255) return false;
  const auto actuator_type = Impl::type(type);
  if (actuator_type == legged::ActuatorType::UNKOWN) {
    std::cerr << "[BumiController] Unknown actuator type: " << type << std::endl;
    return false;
  }
  return impl_->hw->AttachActuator(canu_name, Impl::channel(channel), actuator_type,
                                   actuator_name, static_cast<std::uint8_t>(can_id));
}

bool BumiController::init(const std::string& ifname, std::uint64_t cycle_time_ns,
                          bool enable_dc) {
  return impl_ && impl_->hw && impl_->hw->Init(ifname, cycle_time_ns, enable_dc);
}

bool BumiController::enableAll() {
  return impl_ && impl_->hw && impl_->hw->EnableAllActuator();
}

bool BumiController::disableAll() {
  return impl_ && impl_->hw && impl_->hw->DisableAllActuator();
}

bool BumiController::allActuatorsEnabled() const {
  return impl_ && impl_->hw && impl_->hw->GetAllEnableState();
}




void BumiController::setHeartCnt(std::uint64_t count) {
  if (impl_ && impl_->hw) impl_->hw->SetHeartCnt(count);
}
void BumiController::fetchData() {
  if (impl_ && impl_->hw) impl_->hw->rt_ethercat_get_data();
}
void BumiController::stageCommand() {
  if (impl_ && impl_->hw) impl_->hw->rt_ethercat_set_command();
}
void BumiController::runCycle() {
  if (impl_ && impl_->hw) impl_->hw->rt_ethercat_run();
}

double BumiController::getPosition(const std::string& name) const {
  return impl_ && impl_->hw ? impl_->hw->GetPosition(name) : 0.0;
}
double BumiController::getVelocity(const std::string& name) const {
  return impl_ && impl_->hw ? impl_->hw->GetVelocity(name) : 0.0;
}
double BumiController::getEffort(const std::string& name) const {
  return impl_ && impl_->hw ? impl_->hw->GetEffort(name) : 0.0;
}
bool BumiController::getImuData(const std::string& canu_name, ImuState& imu) const {
  if (!impl_ || !impl_->hw || canu_name.empty()) {
    imu.valid = false;
    return false;
  }

  legged::ImuData raw{};
  if (!impl_->hw->GetImuData(canu_name, raw)) {
    imu.valid = false;
    return false;
  }

  // Bumi vendor data is copied from the CANU receive image populated by
  // rt_ethercat_get_data()/fetchData(); GetImuData itself does not run a
  // second EtherCAT transaction. Keep the public convention [w, x, y, z].
  imu.quaternion = {
      static_cast<float>(raw.ori[0]), static_cast<float>(raw.ori[1]),
      static_cast<float>(raw.ori[2]), static_cast<float>(raw.ori[3])};
  imu.gyroscope = {
      static_cast<float>(raw.angular_vel[0]), static_cast<float>(raw.angular_vel[1]),
      static_cast<float>(raw.angular_vel[2])};
  imu.accelerometer = {
      static_cast<float>(raw.linear_acc[0]), static_cast<float>(raw.linear_acc[1]),
      static_cast<float>(raw.linear_acc[2])};

  bool finite = true;
  for (const float value : imu.quaternion) finite = finite && std::isfinite(value);
  for (const float value : imu.gyroscope) finite = finite && std::isfinite(value);
  for (const float value : imu.accelerometer) finite = finite && std::isfinite(value);
  const double qnorm2 =
      static_cast<double>(imu.quaternion[0]) * imu.quaternion[0] +
      static_cast<double>(imu.quaternion[1]) * imu.quaternion[1] +
      static_cast<double>(imu.quaternion[2]) * imu.quaternion[2] +
      static_cast<double>(imu.quaternion[3]) * imu.quaternion[3];
  imu.valid = finite && qnorm2 > 1.0e-8;
  return imu.valid;
}
void BumiController::setMitCmd(const std::string& name, double pos, double vel,
                               double tau_ff, double kp, double kd) {
  if (!impl_ || !impl_->hw) return;
  impl_->hw->SetMitCmd(name, static_cast<float>(pos), static_cast<float>(vel),
                       static_cast<float>(tau_ff), static_cast<float>(kp),
                       static_cast<float>(kd));
}

}  // namespace noetix
