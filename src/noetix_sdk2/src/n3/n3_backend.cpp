#include "n3_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <unordered_set>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "parallel_ankle.hpp"

namespace noetix
{
namespace
{
constexpr int kFeedbackWarmupCycles = 1000;       // 最多约 1 s @ 1 kHz
constexpr int kEnableConfirmCycles = 3000;        // 最多约 3 s @ 1 kHz
constexpr int kEnableRetryIntervalCycles = 50;    // 每约 50 ms 重发 enable

constexpr double kRawZeroDecodedPosition = -12.5;
constexpr double kRawZeroPositionTolerance = 1.0e-3;

// ActuatorState::error 的本地诊断标志，不对应 vendor error code。
constexpr std::uint32_t kFeedbackInvalidFlag = 0x80000000U;
constexpr std::uint32_t kAnkleInvalidFlag = 0x40000000U;
constexpr float kAnkleMaxFeedforward = 50.0F;
}

struct N3Backend::AnkleRuntime
{
    AnkleRuntime(std::size_t dof, int lp, int lr, int rp, int rr)
        : solver(lp, lr, rp, rr), data(dof) {}

    parallel_ankle::ParallelAnkle solver;
    std::vector<MotorData> data;
};

N3Backend::N3Backend()
    : controller_(std::make_unique<N3Controller>())
{
}

N3Backend::~N3Backend() = default;

bool N3Backend::configure(const RobotConfig& config)
{
    if (started_) {
        std::cerr << "[N3Backend] Cannot configure while running." << std::endl;
        return false;
    }
    if (!controller_) {
        std::cerr << "[N3Backend] Controller is null." << std::endl;
        return false;
    }
    if (config.config_root.empty()) {
        std::cerr << "[N3Backend] config_root is empty." << std::endl;
        return false;
    }
    if (config.actuator_order.empty()) {
        std::cerr << "[N3Backend] Robot layer did not provide actuator_order." << std::endl;
        return false;
    }

    std::string file = config.config_root;
    if (file.back() != '/') file += '/';
    file += "config/hardware.yaml";

    std::cout << "[N3Backend] Loading: " << file << std::endl;

    HardwareConfig parsed;
    parsed.actuator_order = config.actuator_order;

    try {
        const YAML::Node root = YAML::LoadFile(file);
        const YAML::Node runtime = root["hardware"] ? root["hardware"] : root;

        if (runtime["diagnostic_read_only"]) {
            parsed.diagnostic_read_only = runtime["diagnostic_read_only"].as<bool>();
        }


        // 与Bumi一致，底层只读取物理电机标定；robot.yaml的关节坐标映射由上层处理。
        // 两者不能共用一组标定，否则上层和底层的direction会重复应用并相互抵消。
        std::unordered_map<std::string, N3JointCalibration> calib_by_name;
        const YAML::Node actuator_list = root["actuator_list"];
        if (!actuator_list || !actuator_list.IsSequence() || actuator_list.size() == 0U) {
            std::cerr << "[N3Backend] actuator_list[] is required in " << file << std::endl;
            return false;
        }
        for (const auto& row : actuator_list) {
            if (!row.IsMap() || !row["name"] || !row["direction"] || !row["bias"]) {
                std::cerr << "[N3Backend] Incomplete actuator calibration in " << file << std::endl;
                return false;
            }
            const std::string name = row["name"].as<std::string>();
            const int direction = row["direction"].as<int>();
            const double bias = row["bias"].as<double>();
            if (name.empty() || (direction != 1 && direction != -1) || !std::isfinite(bias) ||
                !calib_by_name.emplace(name, N3JointCalibration{direction, bias}).second) {
                std::cerr << "[N3Backend] Invalid/duplicate actuator calibration for "
                          << name << " in " << file << std::endl;
                return false;
            }
        }
        for (const auto& name : parsed.actuator_order) {
            const auto it = calib_by_name.find(name);
            if (it == calib_by_name.end()) {
                std::cerr << "[N3Backend] Missing actuator direction/bias for " << name << std::endl;
                return false;
            }
            parsed.calibration.push_back(it->second);
        }
        if (calib_by_name.size() != parsed.actuator_order.size()) {
            std::cerr << "[N3Backend] Calibration name set differs from robot actuator_order."
                      << std::endl;
            return false;
        }

        // The old N3HW parallel ankle solver is mandatory for this model.
        // Indices are resolved by name: config.actuator_order may be reordered.
        const char* ankle_names[] = {"l_ankle_pitch_joint", "l_ankle_roll_joint",
                                     "r_ankle_pitch_joint", "r_ankle_roll_joint"};
        for (const char* ankle_name : ankle_names) {
            if (calib_by_name.find(ankle_name) == calib_by_name.end()) {
                std::cerr << "[N3Backend] Missing parallel ankle joint: "
                          << ankle_name << std::endl;
                return false;
            }
        }

        if (!root["ethercat"]) {
            std::cerr << "[N3Backend] Missing 'ethercat' node." << std::endl;
            return false;
        }

        const auto e = root["ethercat"];
        if (!e["ifname"] || !e["cycle_time_ns"] || !e["enable_dc"]) {
            std::cerr << "[N3Backend] Incomplete 'ethercat' config." << std::endl;
            return false;
        }

        parsed.ethercat.ifname = e["ifname"].as<std::string>();
        parsed.ethercat.cycle_time_ns = e["cycle_time_ns"].as<std::uint64_t>();
        parsed.ethercat.enable_dc = e["enable_dc"].as<bool>();

        if (parsed.ethercat.ifname.empty() || parsed.ethercat.cycle_time_ns == 0ULL) {
            std::cerr << "[N3Backend] Invalid EtherCAT config." << std::endl;
            return false;
        }

        if (!root["canu_network"]) {
            std::cerr << "[N3Backend] Missing 'canu_network' node." << std::endl;
            return false;
        }

        std::unordered_set<int> ecat_ids;
        std::unordered_set<std::string> topology_names;

        for (const auto& cn : root["canu_network"]) {
            if (cn["enable"] && !cn["enable"].as<bool>()) continue;
            if (!cn["name"] || !cn["ecat_id"]) {
                std::cerr << "[N3Backend] Invalid CANU entry." << std::endl;
                return false;
            }

            CanuConfig canu;
            canu.name = cn["name"].as<std::string>();
            canu.ethercat_id = cn["ecat_id"].as<int>();

            if (!ecat_ids.emplace(canu.ethercat_id).second) {
                std::cerr << "[N3Backend] Duplicate ecat_id: "
                          << canu.ethercat_id << std::endl;
                return false;
            }

            if (cn["imu"] && cn["imu"]["name"]) {
                canu.has_imu = true;
                canu.imu_name = cn["imu"]["name"].as<std::string>();
            }

            const auto parse_channel = [&](const std::string& key, int channel) -> bool {
                if (!cn[key]) return true;

                std::unordered_set<int> can_ids;
                for (const auto& a : cn[key]) {
                    if (!a["name"] || !a["type"] || !a["can_id"]) {
                        std::cerr << "[N3Backend] Invalid actuator entry in "
                                  << canu.name << "/" << key << std::endl;
                        return false;
                    }

                    ActuatorConfig actuator;
                    actuator.name = a["name"].as<std::string>();
                    actuator.type = a["type"].as<std::string>();
                    actuator.channel = channel;
                    actuator.can_id = a["can_id"].as<int>();

                    if (!can_ids.emplace(actuator.can_id).second) {
                        std::cerr << "[N3Backend] Duplicate CAN id " << actuator.can_id
                                  << " in " << canu.name << "/" << key << std::endl;
                        return false;
                    }
                    if (!topology_names.emplace(actuator.name).second) {
                        std::cerr << "[N3Backend] Duplicate actuator name in hardware topology: "
                                  << actuator.name << std::endl;
                        return false;
                    }

                    canu.actuators.push_back(std::move(actuator));
                }
                return true;
            };

            if (!parse_channel("channel_1", 1) ||
                !parse_channel("channel_2", 2) ||
                !parse_channel("channel_3", 3)) {
                return false;
            }

            parsed.canus.push_back(std::move(canu));
        }

        std::unordered_set<std::string> requested;
        for (const auto& name : parsed.actuator_order) {
            if (!requested.emplace(name).second) {
                std::cerr << "[N3Backend] Duplicate actuator in RobotConfig.actuator_order: "
                          << name << std::endl;
                return false;
            }
            if (topology_names.find(name) == topology_names.end()) {
                std::cerr << "[N3Backend] actuator '" << name
                          << "' from robot.yaml is missing from hardware.yaml topology."
                          << std::endl;
                return false;
            }
        }

        if (requested.size() != topology_names.size()) {
            std::cerr << "[N3Backend] Hardware topology actuator count ("
                      << topology_names.size() << ") does not match robot mapping count ("
                      << requested.size() << ")." << std::endl;
            for (const auto& name : topology_names) {
                if (requested.find(name) == requested.end()) {
                    std::cerr << "  hardware-only actuator: " << name << std::endl;
                }
            }
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[N3Backend] YAML parse error: " << e.what() << std::endl;
        return false;
    }

    if (config.dof != 0U && parsed.actuator_order.size() != config.dof) {
        std::cerr << "[N3Backend] DOF mismatch: RobotConfig=" << config.dof
                  << ", actuator_order=" << parsed.actuator_order.size() << std::endl;
        return false;
    }

    hardware_config_ = std::move(parsed);
    dof_ = hardware_config_.actuator_order.size();

    const auto index_of = [this](const char* name) -> int {
        const auto& order = hardware_config_.actuator_order;
        const auto it = std::find(order.begin(), order.end(), name);
        return static_cast<int>(std::distance(order.begin(), it));
    };
    ankle_ = std::make_unique<AnkleRuntime>(
        dof_, index_of("l_ankle_pitch_joint"), index_of("l_ankle_roll_joint"),
        index_of("r_ankle_pitch_joint"), index_of("r_ankle_roll_joint"));
    ankle_feedback_ready_ = false;

    feedback_valid_.assign(dof_, 0U);
    last_valid_state_.assign(dof_, ActuatorState{});

    primary_imu_canu_.clear();
    primary_imu_name_.clear();
    imu_state_ = ImuState{};
    imu_configured_ = false;

    if (!setupHardware()) return false;

    std::cout << "[N3Backend] Hardware topology and direction/bias calibrated for "
              << dof_ << " actuators. N3 parallel ankle enabled." << std::endl;

    configured_ = true;
    return true;
}

bool N3Backend::setupHardware()
{
    if (!controller_) {
        return false;
    }

    for (const auto& canu : hardware_config_.canus) {
        if (!controller_->createCanu(
                canu.name,
                canu.ethercat_id)) {
            std::cerr
                << "[N3Backend] CreateCanu failed: "
                << canu.name
                << std::endl;
            return false;
        }

        for (const auto& actuator : canu.actuators) {
            if (!controller_->attachActuator(
                    canu.name,
                    actuator.channel,
                    actuator.type,
                    actuator.name,
                    actuator.can_id)) {
                std::cerr
                    << "[N3Backend] AttachActuator failed: "
                    << actuator.name
                    << std::endl;
                return false;
            }
        }

        if (canu.has_imu) {
            if (!controller_->attachImu(
                    canu.name,
                    canu.imu_name)) {
                std::cerr
                    << "[N3Backend] AttachImu failed: canu="
                    << canu.name
                    << " imu="
                    << canu.imu_name
                    << std::endl;
                return false;
            }

            if (!imu_configured_) {
                primary_imu_canu_ = canu.name;
                primary_imu_name_ = canu.imu_name;
                imu_configured_ = true;

                std::cout
                    << "[N3Backend] Primary IMU attached: canu="
                    << primary_imu_canu_
                    << " imu="
                    << primary_imu_name_
                    << std::endl;
            }
            else {
                std::cerr
                    << "[N3Backend] Additional IMU attached on CANU "
                    << canu.name
                    << "; LowState will currently publish only the primary IMU on "
                    << primary_imu_canu_
                    << std::endl;
            }
        }
    }

    if (!imu_configured_) {
        std::cerr
            << "[N3Backend] No IMU configured in canu_network."
            << std::endl;
    }

    return true;
}

// SetMitCmd in the original vendor driver may ENABLE a motor even with all
// five MIT fields set to zero. Always queue DISABLE while the SDK is passive.
void N3Backend::queueDisable()
{
    if (controller_) (void)controller_->disableAll();
}

void N3Backend::setZeroMitCommand()
{
    if (!controller_) {
        return;
    }

    for (const auto& name :
         hardware_config_.actuator_order) {
        controller_->setMitCmd(
            name,
            0.0,  // q
            0.0,  // dq
            0.0,  // tau_ff
            0.0,  // kp
            0.0   // kd
        );
    }
}

void N3Backend::startupCycle()
{
    controller_->setHeartCnt(++heart_count_);
    controller_->stageCommand();
    controller_->runCycle();
    controller_->fetchData();

    if (imu_configured_) {
        controller_->getImuData(
            primary_imu_canu_,
            imu_state_);
    }
}

bool N3Backend::feedbackLooksValid(
    double q_motor,
    double dq_motor,
    double tau_motor) const
{
    if (!std::isfinite(q_motor) ||
        !std::isfinite(dq_motor) ||
        !std::isfinite(tau_motor)) {
        return false;
    }

    // Validity must be checked in raw motor coordinates.
    // The vendor's all-zero MIT feedback decodes to q_motor ~= -12.5.
    const bool raw_zero_pattern =
        std::fabs(q_motor - kRawZeroDecodedPosition) <=
        kRawZeroPositionTolerance;

    return !raw_zero_pattern;
}

std::size_t N3Backend::refreshFeedbackValidity(bool verbose)
{
    if (!controller_) {
        return 0;
    }

    std::size_t valid_count = 0;

    for (std::size_t i = 0;
         i < dof_;
         ++i) {

        const auto& name =
            hardware_config_.actuator_order[i];

        const double q_motor =
            controller_->getPosition(name);
        const double dq_motor =
            controller_->getVelocity(name);
        const double tau_motor =
            controller_->getEffort(name);
        const double temperature =
            controller_->getTemperature(name);

        const bool valid =
            feedbackLooksValid(q_motor, dq_motor, tau_motor);

        feedback_valid_[i] =
            valid ? 1U : 0U;

        if (valid) {
            ++valid_count;

            auto& cached =
                last_valid_state_[i];

            // Preserve the original N3HW direction/bias conversions.
            const auto& cal = hardware_config_.calibration[i];
            cached.position = static_cast<float>(q_motor * cal.direction + cal.bias);
            cached.velocity = static_cast<float>(dq_motor * cal.direction);
            cached.torque = static_cast<float>(tau_motor * cal.direction);
            cached.temperature =
                static_cast<float>(temperature);
            cached.error = 0U;
        }
        else if (verbose) {
            std::cerr
                << "[N3Backend] Invalid feedback: "
                << name
                << " | q_motor=" << q_motor
                << " dq_motor=" << dq_motor
                << " tau_motor=" << tau_motor
                << " | error=\""
                << controller_->getErrorString(name)
                << "\""
                << std::endl;
        }
    }

    return valid_count;
}

void N3Backend::dumpInvalidActuators() const
{
    if (!controller_) {
        return;
    }

    for (std::size_t i = 0;
         i < dof_;
         ++i) {

        if (feedback_valid_[i] != 0U) {
            continue;
        }

        const auto& name =
            hardware_config_.actuator_order[i];

        std::cerr
            << "[N3Backend] No valid motor feedback: "
            << name
            << " | q=" << controller_->getPosition(name)
            << " dq=" << controller_->getVelocity(name)
            << " tau=" << controller_->getEffort(name)
            << " | error=\""
            << controller_->getErrorString(name)
            << "\""
            << std::endl;
    }
}

void N3Backend::dumpActuatorDiagnostics() const
{
    if (!controller_) {
        return;
    }

    std::cerr
        << "[N3Backend] Actuator diagnostics:"
        << std::endl;

    for (const auto& name :
         hardware_config_.actuator_order) {
        std::cerr
            << "  " << name
            << " | q=" << controller_->getPosition(name)
            << " dq=" << controller_->getVelocity(name)
            << " tau=" << controller_->getEffort(name)
            << " temp=" << controller_->getTemperature(name)
            << " | error=\""
            << controller_->getErrorString(name)
            << "\""
            << std::endl;
    }
}

bool N3Backend::start()
{
    if (started_) {
        return true;
    }

    if (!configured_ || !controller_) {
        std::cerr
            << "[N3Backend] start() called before configure()."
            << std::endl;
        return false;
    }

    std::cout
        << "[N3Backend] Starting EtherCAT on "
        << hardware_config_.ethercat.ifname
        << " | cycle="
        << hardware_config_.ethercat.cycle_time_ns
        << " ns | dc="
        << (hardware_config_.ethercat.enable_dc
                ? "true"
                : "false")
        << " | diagnostic_read_only="
        << (hardware_config_.diagnostic_read_only
                ? "true"
                : "false")
        << std::endl;

    if (!controller_->init(
            hardware_config_.ethercat.ifname,
            hardware_config_.ethercat.cycle_time_ns,
            hardware_config_.ethercat.enable_dc)) {
        std::cerr
            << "[N3Backend] EtherCAT init failed."
            << std::endl;
        return false;
    }

    const int slave_count =
        controller_->slaveCount();

    const int expected_slaves =
        static_cast<int>(
            hardware_config_.canus.size());

    if (slave_count <= 0) {
        std::cerr
            << "[N3Backend] No EtherCAT slaves found."
            << std::endl;
        return false;
    }

    if (slave_count != expected_slaves) {
        std::cerr
            << "[N3Backend] EtherCAT slave count mismatch: found="
            << slave_count
            << ", expected="
            << expected_slaves
            << std::endl;
        return false;
    }

    if (!controller_->allSlavesOperational()) {
        std::cerr
            << "[N3Backend] EtherCAT slaves are not all OPERATIONAL."
            << std::endl;
        return false;
    }

    std::cout
        << "[N3Backend] EtherCAT topology OK: "
        << slave_count
        << " slaves, all OPERATIONAL."
        << std::endl;

    heart_count_ = 0;
    actuators_enabled_ = false;

    // Original N3 vendor SetMitCmd may implicitly enable a motor.
    // Before explicit enableActuators(), use DisableAllActuator only.
    queueDisable();

    const auto startup_period =
        std::chrono::nanoseconds(
            hardware_config_.ethercat.cycle_time_ns);

    // ----------------------------------------------------------
    // 1. 建立 CANU <-> motor 周期通信。
    // ----------------------------------------------------------
    std::size_t valid_count = 0;

    for (int i = 0;
         i < kFeedbackWarmupCycles;
         ++i) {

        startupCycle();
        valid_count =
            refreshFeedbackValidity(false);

        if (valid_count == dof_) {
            break;
        }

        std::this_thread::sleep_for(
            startup_period);
    }

    valid_count =
        refreshFeedbackValidity(false);

    if (valid_count == 0U) {
        std::cerr
            << "[N3Backend] No actuator has valid feedback. "
            << "Backend cannot enter diagnostic mode."
            << std::endl;

        dumpInvalidActuators();
        return false;
    }

    if (valid_count != dof_) {
        std::cerr
            << "[N3Backend] Partial actuator feedback: "
            << valid_count
            << "/"
            << dof_
            << " valid."
            << std::endl;

        dumpInvalidActuators();

        if (!hardware_config_.diagnostic_read_only) {
            std::cerr
                << "[N3Backend] Normal mode requires all actuator "
                << "feedback. Set diagnostic_read_only: true for "
                << "read-only troubleshooting."
                << std::endl;
            return false;
        }

        std::cerr
            << "[N3Backend] DIAGNOSTIC READ-ONLY MODE ACTIVE. "
            << "Actuators will NOT be enabled and motor "
            << "commands will be forced to DISABLE."
            << std::endl;
    }
    else {
        std::cout
            << "[N3Backend] Feedback OK for all "
            << dof_
            << " actuators."
            << std::endl;
    }

    // A vendor library may implicitly enable on init. Refuse to expose a
    // passive SDK while hardware reports the enabled state.
    if (controller_->allActuatorsEnabled()) {
        std::cerr << "[N3Backend] Drives still enabled after startup DISABLE."
                  << std::endl;
        queueDisable();
        controller_->stageCommand();
        controller_->runCycle();
        return false;
    }

    // ----------------------------------------------------------
    // 2. 诊断模式：无论反馈是否完整都绝不使能。
    // ----------------------------------------------------------
    if (hardware_config_.diagnostic_read_only) {
        queueDisable();
        controller_->setHeartCnt(++heart_count_);
        controller_->stageCommand();
        controller_->runCycle();
        controller_->fetchData();

        started_ = true;

        std::cout
            << "[N3Backend] Started in DIAGNOSTIC READ-ONLY mode. "
            << "valid_feedback="
            << valid_count
            << "/"
            << dof_
            << std::endl;

        return true;
    }

    // ----------------------------------------------------------
    // 3. Communication is now started. Drive enable is a separate
    //    runtime operation controlled by the upper state machine.
    // ----------------------------------------------------------
    started_ = true;

    std::cout << "[N3Backend] Drive starts DISABLED; waiting for runtime ENABLE."
              << std::endl;

    std::cout << "[N3Backend] Started successfully." << std::endl;
    return true;
}

bool N3Backend::enableActuators()
{
    if (!started_ || !controller_) {
        std::cerr << "[N3Backend] enableActuators() requires a started backend."
                  << std::endl;
        return false;
    }
    if (hardware_config_.diagnostic_read_only) {
        std::cerr << "[N3Backend] Refuse ENABLE in diagnostic read-only mode."
                  << std::endl;
        return false;
    }
    if (actuators_enabled_) return true;

    // 使能前确认真实电机反馈存在；不再把踝关节拟合域或位置阈值作为开机条件。
    std::vector<ActuatorState> pre_enable_state(dof_);
    if (!read(pre_enable_state)) {
        std::cerr << "[N3Backend] Refuse ENABLE: motor feedback is invalid." << std::endl;
        dumpInvalidActuators();
        return false;
    }
    // Explicit ENABLE is the only place where the motor command path is opened.
    // Request enable first; SetMitCmd must never be used in passive mode.
    const auto period = std::chrono::nanoseconds(hardware_config_.ethercat.cycle_time_ns);

    bool request_ok = true;
    for (int i = 0; i < kEnableConfirmCycles; ++i) {
        // 原厂ENABLE与MIT共用发送缓冲区：先清零MIT，再填ENABLE，最后发送。
        // 若在enableAll()后再写MIT，首次自动使能计数耗尽后会覆盖显式ENABLE帧。
        setZeroMitCommand();
        if (i == 0 || (i % kEnableRetryIntervalCycles) == 0) {
            if (!controller_->enableAll()) {
                request_ok = false;
                std::cerr << "[N3Backend] EnableAllActuator() request failed." << std::endl;
                break;
            }
        }
        startupCycle();
        if (controller_->allActuatorsEnabled()) {
            actuators_enabled_ = true;
            break;
        }
        std::this_thread::sleep_for(period);
    }

    if (!request_ok || !actuators_enabled_) {
        std::cerr << "[N3Backend] Runtime ENABLE was not confirmed by all actuators."
                  << std::endl;
        dumpActuatorDiagnostics();
        queueDisable();
        controller_->stageCommand();
        controller_->runCycle();
        actuators_enabled_ = false;
        return false;
    }

    setZeroMitCommand();
    startupCycle();
    std::cout << "[N3Backend] All actuators ENABLED and confirmed." << std::endl;
    return true;
}

bool N3Backend::disableActuators()
{
    if (!started_ || !controller_) return false;
    if (!actuators_enabled_) {
        // Idempotent passive call. Still refresh the queued DISABLE request.
        queueDisable();
        controller_->stageCommand();
        controller_->runCycle();
        return true;
    }

    // Never call SetMitCmd after requesting disable. Vendor MIT setter may
    // implicitly re-enable a drive, even when every command field is zero.
    const bool request_ok = controller_->disableAll();
    const auto period = std::chrono::nanoseconds(hardware_config_.ethercat.cycle_time_ns);
    for (int i = 0; i < 10; ++i) {
        controller_->setHeartCnt(++heart_count_);
        controller_->stageCommand();
        controller_->runCycle();
        controller_->fetchData();
        std::this_thread::sleep_for(period);
    }
    actuators_enabled_ = false;
    if (!request_ok) {
        std::cerr << "[N3Backend] DisableAllActuator() request failed." << std::endl;
    }
    return request_ok;
}

bool N3Backend::stop()
{
    if (!controller_) {
        started_ = false;
        actuators_enabled_ = false;
        return false;
    }
    if (!started_) return true;

    // Stop must not send MIT: zero-MIT can itself trigger enable.
    const bool disable_ok = disableActuators();
    started_ = false;
    actuators_enabled_ = false;
    return disable_ok;
}

bool N3Backend::read(
    std::vector<ActuatorState>& state)
{
    if (!started_ || !controller_) {
        return false;
    }

    if (state.size() != dof_) {
        // 正常路径 configure() 后尺寸已经固定。
        // 此分支仅防御，不应在 RT 周期中触发。
        state.resize(dof_);
    }

    // 原厂 N3HW read 语义：
    // heartbeat -> decode previously received PDO -> read actuator state。
    controller_->setHeartCnt(++heart_count_);
    controller_->fetchData();

    // IMU is read from the same PDO snapshot as the actuator feedback.
    if (imu_configured_) {
        controller_->getImuData(
            primary_imu_canu_,
            imu_state_);
    }
    else {
        imu_state_.valid = false;
    }

    std::size_t valid_count = 0;

    for (std::size_t i = 0;
         i < dof_;
         ++i) {

        const auto& name =
            hardware_config_.actuator_order[i];

        const double q_motor =
            controller_->getPosition(name);
        const double dq_motor =
            controller_->getVelocity(name);
        const double tau_motor =
            controller_->getEffort(name);
        const double temperature =
            controller_->getTemperature(name);

        const bool valid =
            feedbackLooksValid(q_motor, dq_motor, tau_motor);

        feedback_valid_[i] =
            valid ? 1U : 0U;

        if (valid) {
            ++valid_count;

            auto& cached =
                last_valid_state_[i];

            // Preserve the original N3HW direction/bias conversions.
            const auto& cal = hardware_config_.calibration[i];
            cached.position = static_cast<float>(q_motor * cal.direction + cal.bias);
            cached.velocity = static_cast<float>(dq_motor * cal.direction);
            cached.torque = static_cast<float>(tau_motor * cal.direction);
            cached.temperature =
                static_cast<float>(temperature);
            cached.error = 0U;

            state[i] = cached;
        }
        else {
            // 诊断模式下不能把 -12.5/-18/-150 之类的协议下限
            // 直接发布给 robot_state_publisher。
            // 使用最后一次有效位置；若从未有效，configure() 初值为 0 rad。
            state[i] =
                last_valid_state_[i];

            state[i].velocity = 0.0F;
            state[i].torque = 0.0F;
            state[i].error |=
                kFeedbackInvalidFlag;
        }
    }

    // Old N3HW converts the calibrated ankle motor states into task-space
    // pitch/roll using FK + Jacobian. This has to run before the controller
    // sees LowState, and before the same-cycle command transform.
    for (std::size_t i = 0; i < dof_; ++i) {
        auto& joint = ankle_->data[i];
        joint.pos_ = state[i].position;
        joint.vel_ = state[i].velocity;
        joint.tau_ = state[i].torque;
    }
    ankle_->solver.parallel_ankle_solve_state(ankle_->data);
    ankle_feedback_ready_ = ankle_->solver.is_motor_ready();

    for (const char* name : {"l_ankle_pitch_joint", "l_ankle_roll_joint",
                             "r_ankle_pitch_joint", "r_ankle_roll_joint"}) {
        const auto it = std::find(hardware_config_.actuator_order.begin(),
                                  hardware_config_.actuator_order.end(), name);
        const std::size_t idx = static_cast<std::size_t>(
            std::distance(hardware_config_.actuator_order.begin(), it));
        state[idx].position = static_cast<float>(ankle_->data[idx].pos_);
        state[idx].velocity = static_cast<float>(ankle_->data[idx].vel_);
        state[idx].torque = static_cast<float>(ankle_->data[idx].tau_);
        if (!ankle_feedback_ready_) state[idx].error |= kAnkleInvalidFlag;
    }

    if (hardware_config_.diagnostic_read_only) {
        // 只要 EtherCAT backend 本身在线，允许部分反馈继续发布。
        return valid_count > 0U;
    }

    // Fail closed on a missing actuator. Do not allow the next policy frame
    // to reuse a stale MIT command while the robot is partially disconnected.
    if (valid_count != dof_ && actuators_enabled_) {
        queueDisable();
        controller_->stageCommand();
        controller_->runCycle();
        actuators_enabled_ = false;
    }
    return valid_count == dof_;
}

bool N3Backend::readImu(ImuState& imu)
{
    if (!started_ || !imu_configured_ || !imu_state_.valid) {
        imu.valid = false;
        return false;
    }

    imu = imu_state_;
    return true;
}

bool N3Backend::write(const std::vector<ActuatorCmd>& command)
{
    if (!started_ || !controller_) return false;

    if (hardware_config_.diagnostic_read_only || !actuators_enabled_) {
        // A zero SetMitCmd is NOT a safe passive command with this vendor.
        queueDisable();
        controller_->stageCommand();
        controller_->runCycle();
        return true;
    }

    if (command.size() != dof_) {
        queueDisable();
        actuators_enabled_ = false;
        controller_->stageCommand();
        controller_->runCycle();
        return false;
    }
    for (std::size_t i = 0; i < dof_; ++i) {
        const auto& cmd = command[i];
        if (cmd.name != hardware_config_.actuator_order[i] ||
            !std::isfinite(cmd.position) || !std::isfinite(cmd.velocity) ||
            !std::isfinite(cmd.torque_ff) || !std::isfinite(cmd.kp) ||
            !std::isfinite(cmd.kd) || cmd.kp < 0.0 || cmd.kd < 0.0) {
            queueDisable();
            actuators_enabled_ = false;
            controller_->stageCommand();
            controller_->runCycle();
            return false;
        }
        auto& j = ankle_->data[i];
        j.pos_des_ = cmd.position;
        j.vel_des_ = cmd.velocity;
        j.ff_ = cmd.torque_ff;
        j.kp_ = cmd.kp;
        j.kd_ = cmd.kd;
    }

    // 并联踝把任务空间指令转换为电机MIT力位混合指令：
    // IK求目标角，逆雅可比求目标速度，JᵀKJ/JᵀDJ求增益，耦合项并入前馈。
    ankle_->solver.parallel_ankle_solve_cmd(ankle_->data);
    for (std::size_t i = 0; i < dof_; ++i) {
        const auto& cmd = command[i];
        const auto& name = hardware_config_.actuator_order[i];
        const auto& cal = hardware_config_.calibration[i];
        const bool is_ankle =
            name == "l_ankle_pitch_joint" || name == "l_ankle_roll_joint" ||
            name == "r_ankle_pitch_joint" || name == "r_ankle_roll_joint";
        if (is_ankle) {
            // 保留已有反馈保护；无效时目标、增益与前馈一起清零。
            if (!ankle_feedback_ready_) {
                controller_->setMitCmd(name, 0.0, 0.0, 0.0, 0.0, 0.0);
                continue;
            }
            const auto& motor = ankle_->data[i];
            const float ff = std::clamp(static_cast<float>(motor.ff_),
                                       -kAnkleMaxFeedforward, kAnkleMaxFeedforward);
            // 解算结果为标定后的电机坐标；下发前只做一次硬件标定反变换。
            controller_->setMitCmd(
                name, (motor.pos_des_ - cal.bias) * cal.direction,
                motor.vel_des_ * cal.direction, ff * cal.direction,
                motor.kp_, motor.kd_);
        } else {
            controller_->setMitCmd(
                name, (cmd.position - cal.bias) * cal.direction,
                cmd.velocity * cal.direction, cmd.torque_ff * cal.direction,
                cmd.kp, cmd.kd);
        }
    }

    controller_->stageCommand();
    controller_->runCycle();
    return true;
}

}  // namespace noetix
