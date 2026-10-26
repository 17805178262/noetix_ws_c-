#include "n3_controller.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include "common_type.h"
#include "hw_controller.h"
#include "soem/ethercat.h"

namespace noetix
{

class N3Controller::Impl
{
public:
    legged::HwController* hw{nullptr};

    static legged::CtrlChannel toChannel(int channel)
    {
        switch (channel) {
            case 1: return legged::CtrlChannel::CTRL_CH1;
            case 2: return legged::CtrlChannel::CTRL_CH2;
            case 3: return legged::CtrlChannel::CTRL_CH3;
            default: return legged::CtrlChannel::CTRL_CH1;
        }
    }

    static legged::ActuatorType toType(const std::string& type)
    {
        if (type == "ACTR_5014") return legged::ActuatorType::ACTR_5014;
        if (type == "ACTR_8522") return legged::ActuatorType::ACTR_8522;
        if (type == "ACTR_4308") return legged::ActuatorType::ACTR_4308;
        return legged::ActuatorType::UNKOWN;
    }
};

N3Controller::N3Controller()
    : impl_(new Impl())
{
    impl_->hw = legged::HwController::GetInstance();
}

N3Controller::~N3Controller()
{
    delete impl_;
    impl_ = nullptr;
}

bool N3Controller::init(const std::string& ifname,
                        std::uint64_t cycle_time_ns,
                        bool enable_dc)
{
    if (!impl_ || !impl_->hw) return false;
    return impl_->hw->Init(ifname, cycle_time_ns, enable_dc);
}

bool N3Controller::createCanu(const std::string& name,
                              int ecat_id)
{
    if (!impl_ || !impl_->hw) return false;
    if (ecat_id < 0 || ecat_id > 255) return false;

    return impl_->hw->CreateCanu(
        name,
        static_cast<std::uint8_t>(ecat_id));
}

bool N3Controller::attachActuator(const std::string& canu_name,
                                  int channel,
                                  const std::string& type,
                                  const std::string& actuator_name,
                                  int can_id)
{
    if (!impl_ || !impl_->hw) return false;
    if (channel < 1 || channel > 3) return false;
    if (can_id < 0 || can_id > 255) return false;

    const auto actuator_type = Impl::toType(type);
    if (actuator_type == legged::ActuatorType::UNKOWN) {
        std::cerr << "[N3Controller] Unknown actuator type: "
                  << type << std::endl;
        return false;
    }

    return impl_->hw->AttachActuator(
        canu_name,
        Impl::toChannel(channel),
        actuator_type,
        actuator_name,
        static_cast<std::uint8_t>(can_id));
}

bool N3Controller::attachImu(const std::string& canu_name,
                             const std::string& imu_name)
{
    if (!impl_ || !impl_->hw) return false;
    if (canu_name.empty() || imu_name.empty()) return false;

    return impl_->hw->AttachImu(canu_name, imu_name);
}

bool N3Controller::getImuData(const std::string& canu_name,
                              ImuState& imu) const
{
    if (!impl_ || !impl_->hw || canu_name.empty()) {
        imu.valid = false;
        return false;
    }

    legged::ImuRawData raw{};
    if (!impl_->hw->GetImuData(canu_name, raw)) {
        imu.valid = false;
        return false;
    }

    // Vendor / original N3HW semantics:
    // raw.quat = [w, x, y, z].
    imu.quaternion[0] = raw.quat[0];
    imu.quaternion[1] = raw.quat[1];
    imu.quaternion[2] = raw.quat[2];
    imu.quaternion[3] = raw.quat[3];

    imu.gyroscope[0] = raw.gyro[0];
    imu.gyroscope[1] = raw.gyro[1];
    imu.gyroscope[2] = raw.gyro[2];

    // Keep the active original N3HW behavior: copy acc directly.
    // Do not multiply by 9.81 here unless the firmware is verified to output g.
    imu.accelerometer[0] = raw.acc[0];
    imu.accelerometer[1] = raw.acc[1];
    imu.accelerometer[2] = raw.acc[2];

    bool finite = true;
    for (float v : imu.quaternion) finite = finite && std::isfinite(v);
    for (float v : imu.gyroscope) finite = finite && std::isfinite(v);
    for (float v : imu.accelerometer) finite = finite && std::isfinite(v);

    const double qnorm2 =
        static_cast<double>(imu.quaternion[0]) * imu.quaternion[0] +
        static_cast<double>(imu.quaternion[1]) * imu.quaternion[1] +
        static_cast<double>(imu.quaternion[2]) * imu.quaternion[2] +
        static_cast<double>(imu.quaternion[3]) * imu.quaternion[3];

    imu.valid = finite && qnorm2 > 1.0e-8;
    return imu.valid;
}

bool N3Controller::enableAll()
{
    if (!impl_ || !impl_->hw) return false;
    return impl_->hw->EnableAllActuator();
}

bool N3Controller::disableAll()
{
    if (!impl_ || !impl_->hw) return false;
    return impl_->hw->DisableAllActuator();
}

bool N3Controller::allActuatorsEnabled() const
{
    if (!impl_ || !impl_->hw) return false;
    return impl_->hw->GetAllEnableState();
}

int N3Controller::slaveCount() const
{
    return ec_slavecount;
}

bool N3Controller::allSlavesOperational() const
{
    if (ec_slavecount <= 0) {
        return false;
    }

    // Refresh every slave AL state before inspecting ec_slave[i].state.
    ec_readstate();

    bool all_operational = true;

    for (int i = 1; i <= ec_slavecount; ++i) {
        const std::uint16_t state =
            static_cast<std::uint16_t>(ec_slave[i].state);

        const std::uint16_t base_state = state & 0x0FU;
        const bool has_error = (state & EC_STATE_ERROR) != 0U;

        if (base_state != EC_STATE_OPERATIONAL || has_error) {
            all_operational = false;

            std::cerr
                << "[N3Controller] Slave " << i
                << " is not OPERATIONAL"
                << " | state=0x" << std::hex
                << static_cast<unsigned int>(state)
                << " | ALstatus=0x"
                << static_cast<unsigned int>(ec_slave[i].ALstatuscode)
                << std::dec << std::endl;
        }
    }

    return all_operational;
}

void N3Controller::setHeartCnt(std::uint64_t count)
{
    if (!impl_ || !impl_->hw) return;
    impl_->hw->SetHeartCnt(count);
}

void N3Controller::fetchData()
{
    if (!impl_ || !impl_->hw) return;
    impl_->hw->rt_ethercat_get_data();
}

void N3Controller::stageCommand()
{
    if (!impl_ || !impl_->hw) return;
    impl_->hw->rt_ethercat_set_command();
}

void N3Controller::runCycle()
{
    if (!impl_ || !impl_->hw) return;
    impl_->hw->rt_ethercat_run();
}

double N3Controller::getPosition(const std::string& name) const
{
    if (!impl_ || !impl_->hw) return 0.0;
    return impl_->hw->GetPosition(name);
}

double N3Controller::getVelocity(const std::string& name) const
{
    if (!impl_ || !impl_->hw) return 0.0;
    return impl_->hw->GetVelocity(name);
}

double N3Controller::getEffort(const std::string& name) const
{
    if (!impl_ || !impl_->hw) return 0.0;
    return impl_->hw->GetEffort(name);
}

double N3Controller::getTemperature(const std::string& name) const
{
    if (!impl_ || !impl_->hw) return 0.0;
    return static_cast<double>(impl_->hw->GetTemperature(name));
}

std::string N3Controller::getErrorString(const std::string& name) const
{
    if (!impl_ || !impl_->hw) {
        return "HwController unavailable";
    }

    return impl_->hw->GetErrorString(name);
}

void N3Controller::setMitCmd(const std::string& name,
                             double pos,
                             double vel,
                             double tau_ff,
                             double kp,
                             double kd)
{
    if (!impl_ || !impl_->hw) return;

    impl_->hw->SetMitCmd(
        name,
        static_cast<float>(pos),
        static_cast<float>(vel),
        static_cast<float>(tau_ff),
        static_cast<float>(kp),
        static_cast<float>(kd));
}

}  // namespace noetix
