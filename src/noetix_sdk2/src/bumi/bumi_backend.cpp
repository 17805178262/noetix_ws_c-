#include "noetix/hardware/bumi/bumi_backend.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <filesystem>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "noetix/hardware/bumi/bumi_controller.hpp"
#include "usb_imu_reader.hpp"
#include "parallel_solve/parallel_ankle.h"

namespace noetix {
namespace {
constexpr std::uint32_t kFeedbackInvalidFlag = 0x80000000U;
constexpr double kRawZeroDecodedPosition = -12.5;
constexpr double kRawZeroPositionTolerance = 1.0e-3;
constexpr int kPrimeCycles = 20;
constexpr int kEnableConfirmCycles = 2000;
constexpr int kEnableRetryCycles = 50;
constexpr int kDisableConfirmCycles = 500;
constexpr int kDisableRetryCycles = 25;

struct MotorCalibration {
  int direction{1};
  double bias{0.0};
};

struct TopologyActuator {
  std::string name;
  std::string type;
  int channel{0};
  int can_id{0};
};

struct Canu {
  std::string name;
  int ecat_id{0};
  std::vector<TopologyActuator> actuators;
};

struct AnklePair {
  std::string pitch_name;
  std::string roll_name;
  std::size_t pitch{0};
  std::size_t roll{0};
  int which_leg{0};
};

bool finiteState(double q, double dq, double tau) {
  if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau)) return false;
  // Vendor protocol decodes an all-zero/uninitialized MIT position frame as -12.5 rad.
  return std::fabs(q - kRawZeroDecodedPosition) > kRawZeroPositionTolerance;
}
}  // namespace

class BumiBackend::Impl {
 public:
  BumiController controller;
  std::string ifname;
  std::uint64_t cycle_ns{1000000ULL};
  bool enable_dc{true};
  std::string vendor_robot_model{"bumi1"};
  std::string robot_name;
  std::string network_config_path;
  bool use_usb_imu{false};
  UsbImuConfig usb_imu_config{};
  UsbImuReader usb_imu;
  std::string selected_imu_canu;
  bool configured{false};
  bool started{false};
  bool enabled{false};
  bool ankle_ready{false};
  std::uint64_t heart{0};
  double ankle_motor_offset{0.2618};

  std::vector<std::string> order;
  std::unordered_map<std::string, std::size_t> index;
  std::unordered_map<std::string, MotorCalibration> calibration;
  std::vector<Canu> canus;
  std::array<AnklePair, 2> ankles;

  std::vector<ActuatorState> raw_corrected;
  std::vector<ActuatorState> last_good_raw;
  std::vector<std::uint8_t> has_last_good;
  std::vector<ActuatorState> logical_state;

  void zeroCommands() {
    for (const auto& name : order) controller.setMitCmd(name, 0, 0, 0, 0, 0);
  }

  void exchangeZero() {
    controller.setHeartCnt(++heart);
    zeroCommands();
    controller.stageCommand();
    controller.runCycle();
    controller.fetchData();
  }

  bool load(const RobotConfig& config) {
    if (config.config_root.empty() || config.actuator_order.empty()) return false;
    const std::string path = config.config_root + "/config/hardware.yaml";
    try {
      const YAML::Node root = YAML::LoadFile(path);
      const YAML::Node runtime = root["hardware"] ? root["hardware"] : root;
      if (runtime["vendor_robot_model"]) {
        vendor_robot_model = runtime["vendor_robot_model"].as<std::string>();
      }
      if (vendor_robot_model != "bumi1" && vendor_robot_model != "bumi2") {
        throw std::runtime_error("hardware.vendor_robot_model must be bumi1 or bumi2");
      }

      // All Bumi variants intentionally share one EtherCAT transport file.
      // Model-specific hardware.yaml contains only actuator topology/calibration.
      // This makes bumi3 (known-good), bumi4340 and bumi2 enter the vendor
      // controller through exactly the same network path.
      robot_name = config.robot_name;
      const std::filesystem::path robot_root(config.config_root);
      const std::filesystem::path network_path =
          robot_root.parent_path() / "common" / "bumi" / "network.yaml";
      network_config_path = network_path.string();
      const YAML::Node network_root = YAML::LoadFile(network_config_path);
      const YAML::Node ec = network_root["ethercat"];
      if (!ec || !ec["ifname"]) {
        throw std::runtime_error("common/bumi/network.yaml requires ethercat.ifname");
      }
      ifname = ec["ifname"].as<std::string>();
      if (ifname.empty()) throw std::runtime_error("shared Bumi ethercat.ifname is empty");
      if (ec["cycle_time_ns"]) cycle_ns = ec["cycle_time_ns"].as<std::uint64_t>();
      if (ec["enable_dc"]) enable_dc = ec["enable_dc"].as<bool>();
      if (cycle_ns == 0ULL) throw std::runtime_error("shared Bumi cycle_time_ns must be > 0");

      // CAT is always the default. USB is enabled only by an explicit
      // imu.usb.enable: true in the shared Bumi network configuration.
      use_usb_imu = false;
      selected_imu_canu.clear();
      const YAML::Node imu = network_root["imu"];
      if (imu && imu["usb"]) {
        const YAML::Node usb = imu["usb"];
        const bool enabled = usb["enable"] && usb["enable"].as<bool>();
        if (enabled) {
          if (!usb["port"]) {
            throw std::runtime_error("imu.usb.port is required when imu.usb.enable=true");
          }
          usb_imu_config.port = usb["port"].as<std::string>();
          if (usb_imu_config.port.empty()) {
            throw std::runtime_error("imu.usb.port must not be empty");
          }
          use_usb_imu = true;
        }
      }

      order = config.actuator_order;
      index.clear();
      for (std::size_t i = 0; i < order.size(); ++i) {
        if (!index.emplace(order[i], i).second) throw std::runtime_error("duplicate actuator_order: " + order[i]);
      }

      const YAML::Node acts = root["actuator_list"];
      if (!acts || !acts.IsSequence()) throw std::runtime_error("actuator_list[] is required");
      calibration.clear();
      for (const auto& node : acts) {
        const std::string name = node["name"].as<std::string>();
        MotorCalibration c;
        c.direction = node["direction"].as<int>();
        c.bias = node["bias"].as<double>();
        if ((c.direction != 1 && c.direction != -1) || !std::isfinite(c.bias))
          throw std::runtime_error("invalid calibration: " + name);
        if (!calibration.emplace(name, c).second) throw std::runtime_error("duplicate actuator_list: " + name);
      }
      for (const auto& name : order) {
        if (calibration.find(name) == calibration.end()) throw std::runtime_error("missing calibration: " + name);
      }
      if (calibration.size() != order.size()) throw std::runtime_error("actuator_list and robot mapping size mismatch");

      canus.clear();
      std::unordered_set<std::string> attached;
      const YAML::Node net = root["canu_network"];
      if (!net || !net.IsSequence()) throw std::runtime_error("canu_network[] is required");
      for (const auto& node : net) {
        if (node["enable"] && !node["enable"].as<bool>()) continue;
        Canu canu;
        canu.name = node["name"].as<std::string>();
        canu.ecat_id = node["ecat_id"].as<int>();
        for (int ch = 1; ch <= 3; ++ch) {
          const YAML::Node list = node["channel_" + std::to_string(ch)];
          if (!list) continue;
          for (const auto& a : list) {
            TopologyActuator item;
            item.name = a["name"].as<std::string>();
            item.type = a["type"].as<std::string>();
            item.channel = ch;
            item.can_id = a["can_id"].as<int>();
            if (index.find(item.name) == index.end()) throw std::runtime_error("topology has unknown actuator: " + item.name);
            if (!attached.emplace(item.name).second) throw std::runtime_error("actuator attached twice: " + item.name);
            canu.actuators.push_back(std::move(item));
          }
        }
        canus.push_back(std::move(canu));
      }
      if (attached.size() != order.size()) throw std::runtime_error("canu_network does not attach all actuators");

      const YAML::Node ankle = root["parallel_ankle"];
      if (!ankle) throw std::runtime_error("parallel_ankle is required for Bumi");
      if (ankle["motor_offset_rad"]) ankle_motor_offset = ankle["motor_offset_rad"].as<double>();
      auto loadAnkle = [&](int leg, const char* key) {
        const YAML::Node a = ankle[key];
        ankles[leg].pitch_name = a["pitch"].as<std::string>();
        ankles[leg].roll_name = a["roll"].as<std::string>();
        ankles[leg].pitch = index.at(ankles[leg].pitch_name);
        ankles[leg].roll = index.at(ankles[leg].roll_name);
        ankles[leg].which_leg = leg;
      };
      loadAnkle(0, "left");
      loadAnkle(1, "right");

      raw_corrected.assign(order.size(), ActuatorState{});
      last_good_raw.assign(order.size(), ActuatorState{});
      has_last_good.assign(order.size(), 0U);
      logical_state.assign(order.size(), ActuatorState{});
    } catch (const std::exception& e) {
      std::cerr << "[BumiBackend] Invalid " << path << ": " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool setup() {
    std::cout << "[BumiBackend] robot=" << robot_name
              << " network=" << network_config_path
              << " ifname=" << ifname
              << " cycle_ns=" << cycle_ns
              << " dc=" << (enable_dc ? "true" : "false")
              << " vendor_model=" << vendor_robot_model
              << " canu_count=" << canus.size() << std::endl;

    for (const auto& canu : canus) {
      std::cout << "[BumiBackend] CANU name=" << canu.name
                << " ecat_id=" << canu.ecat_id
                << " actuators=" << canu.actuators.size() << std::endl;
      if (!controller.createCanu(canu.name, canu.ecat_id, vendor_robot_model)) {
        std::cerr << "[BumiBackend] CreateCanu failed: " << canu.name << std::endl;
        return false;
      }
      for (const auto& a : canu.actuators) {
        if (!controller.attachActuator(canu.name, a.channel, a.type, a.name, a.can_id)) {
          std::cerr << "[BumiBackend] AttachActuator failed: " << a.name << std::endl;
          return false;
        }
      }
    }
    if (!controller.init(ifname, cycle_ns, enable_dc)) {
      std::cerr << "[BumiBackend] EtherCAT init failed: robot=" << robot_name
                << " shared_ifname=" << ifname
                << " network=" << network_config_path << std::endl;
      return false;
    }
    std::cout << "[BumiBackend] EtherCAT transport initialized from shared Bumi network config."
              << std::endl;
    return true;
  }

  void solveAnkleState(std::vector<ActuatorState>& state) {
    ankle_ready = true;
    for (const auto& a : ankles) {
      // Bumi parallel-ankle convention: motor-1 is roll slot, motor-2 is pitch slot.
      const double q1 = state[a.roll].position + ankle_motor_offset;
      const double q2 = state[a.pitch].position + ankle_motor_offset;
      const double dq1 = state[a.roll].velocity;
      const double dq2 = state[a.pitch].velocity;
      const double tau1 = state[a.roll].torque;
      const double tau2 = state[a.pitch].torque;

      if (std::fabs(state[a.roll].position) > 3.14 ||
          std::fabs(state[a.pitch].position) > 3.14 ||
          (state[a.roll].error & kFeedbackInvalidFlag) != 0U ||
          (state[a.pitch].error & kFeedbackInvalidFlag) != 0U) {
        ankle_ready = false;
      }

      double roll = 0, pitch = 0, droll = 0, dpitch = 0, troll = 0, tpitch = 0;
      double jac[4]{};
      parallel_ankle_fk(q1, q2, static_cast<double>(a.which_leg), &roll, &pitch);
      parallel_ankle_jacobian(roll, pitch, q1, q2, static_cast<double>(a.which_leg), 0, jac);
      parallel_ankle_fk_vel(dq1, dq2, jac, static_cast<double>(a.which_leg), &droll, &dpitch);
      parallel_ankle_fd(tau1, tau2, jac, static_cast<double>(a.which_leg), &troll, &tpitch);

      state[a.roll].position = static_cast<float>(roll);
      state[a.roll].velocity = static_cast<float>(droll);
      state[a.roll].torque = static_cast<float>(troll);
      state[a.pitch].position = static_cast<float>(pitch);
      state[a.pitch].velocity = static_cast<float>(dpitch);
      state[a.pitch].torque = static_cast<float>(tpitch);
    }
  }

  void commandAnkle(const AnklePair& a, const std::vector<ActuatorCmd>& command) {
    if (!ankle_ready) {
      controller.setMitCmd(a.pitch_name, 0, 0, 0, 0, 0);
      controller.setMitCmd(a.roll_name, 0, 0, 0, 0, 0);
      return;
    }

    const auto& cp = command[a.pitch];
    const auto& cr = command[a.roll];
    const auto& sp = logical_state[a.pitch];
    const auto& sr = logical_state[a.roll];

    const double tau_pitch = cp.kp * (cp.position - sp.position) +
                             cp.kd * (cp.velocity - sp.velocity) + cp.torque_ff;
    const double tau_roll = cr.kp * (cr.position - sr.position) +
                            cr.kd * (cr.velocity - sr.velocity) + cr.torque_ff;

    double q1 = 0, q2 = 0, jac[4]{}, tau1 = 0, tau2 = 0;
    parallel_ankle_ik(sr.position, sp.position, static_cast<double>(a.which_leg), 0, &q1, &q2);
    parallel_ankle_jacobian(sr.position, sp.position, q1, q2,
                            static_cast<double>(a.which_leg), 0, jac);
    parallel_ankle_id(tau_roll, tau_pitch, jac, static_cast<double>(a.which_leg), &tau1, &tau2);

    // q1/tau1 -> roll motor; q2/tau2 -> pitch motor. Hardware direction is applied last.
    const auto& croll = calibration.at(a.roll_name);
    const auto& cpitch = calibration.at(a.pitch_name);
    controller.setMitCmd(a.roll_name, 0, 0, tau1 * croll.direction, 0, 0);
    controller.setMitCmd(a.pitch_name, 0, 0, tau2 * cpitch.direction, 0, 0);
  }
};

BumiBackend::BumiBackend() : impl_(std::make_unique<Impl>()) {}
BumiBackend::~BumiBackend() = default;

bool BumiBackend::configure(const RobotConfig& config) {
  if (!impl_ || impl_->started) return false;
  if (!impl_->load(config) || !impl_->setup()) return false;
  impl_->configured = true;
  return true;
}

bool BumiBackend::start() {
  if (!impl_ || !impl_->configured || impl_->started) return impl_ && impl_->started;
  for (int i = 0; i < kPrimeCycles; ++i) {
    impl_->exchangeZero();
    std::this_thread::sleep_for(std::chrono::nanoseconds(impl_->cycle_ns));
  }
  // HwController::Init already performs slave discovery, PDO mapping and the
  // transition to OPERATIONAL. Do not run a second SOEM-global state check in
  // the SDK; the vendor controller is the single EtherCAT authority.
  impl_->started = true;
  if (impl_->use_usb_imu) {
    if (!impl_->usb_imu.start(impl_->usb_imu_config)) {
      std::cerr << "[BumiBackend] USB IMU start failed: "
                << impl_->usb_imu_config.port << std::endl;
      impl_->started = false;
      return false;
    }
    std::cout << "[BumiBackend] IMU source=usb port="
              << impl_->usb_imu_config.port << " baud=921600" << std::endl;
  } else {
    std::cout << "[BumiBackend] IMU source=ethercat" << std::endl;
  }
  std::cout << "[BumiBackend] Drive starts DISABLED; waiting for runtime ENABLE." << std::endl;
  return true;
}

bool BumiBackend::stop() {
  if (!impl_) return false;
  impl_->usb_imu.stop();
  if (impl_->configured) {
    for (int i = 0; i < 10; ++i) impl_->exchangeZero();
    impl_->controller.disableAll();
  }
  const bool still_enabled = impl_->controller.allActuatorsEnabled();
  impl_->started = false;
  impl_->enabled = still_enabled;
  if (still_enabled) {
    std::cerr << "[BumiBackend] STOP sent zero MIT commands, but actuators remain enabled; "
                 "the vendor disable request is unavailable or failed." << std::endl;
  }
  return !still_enabled;
}

bool BumiBackend::read(std::vector<ActuatorState>& state) {
  if (!impl_ || !impl_->started || state.size() != impl_->order.size()) return false;
  impl_->controller.setHeartCnt(++impl_->heart);
  impl_->controller.fetchData();

  bool all_valid = true;
  for (std::size_t i = 0; i < impl_->order.size(); ++i) {
    const auto& name = impl_->order[i];
    const auto& cal = impl_->calibration.at(name);
    const double q_raw = impl_->controller.getPosition(name);
    const double dq_raw = impl_->controller.getVelocity(name);
    const double tau_raw = impl_->controller.getEffort(name);
    const bool valid = finiteState(q_raw, dq_raw, tau_raw);
    all_valid = all_valid && valid;

    ActuatorState sample{};
    if (valid) {
      sample.position = static_cast<float>(q_raw * cal.direction + cal.bias);
      sample.velocity = static_cast<float>(dq_raw * cal.direction);
      sample.torque = static_cast<float>(tau_raw * cal.direction);
      impl_->last_good_raw[i] = sample;
      impl_->has_last_good[i] = 1U;
    } else if (impl_->has_last_good[i] != 0U) {
      sample = impl_->last_good_raw[i];
      sample.error |= kFeedbackInvalidFlag;
    } else {
      sample.error = kFeedbackInvalidFlag;
    }
    impl_->raw_corrected[i] = sample;
  }

  if (!all_valid) return false;

  impl_->logical_state = impl_->raw_corrected;
  impl_->solveAnkleState(impl_->logical_state);
  state = impl_->logical_state;
  return true;
}

bool BumiBackend::readImu(ImuState& imu) {
  if (!impl_ || !impl_->started) {
    imu.valid = false;
    return false;
  }

  if (impl_->use_usb_imu) {
    return impl_->usb_imu.read(imu);
  }

  // fetchData() has already updated every CANU receive image in read().
  // GetImuData only copies that cached image, so the motor EtherCAT read/write
  // sequence remains the original one. Try the last valid CANU first, then
  // discover the IMU automatically from the configured CANUs.
  if (!impl_->selected_imu_canu.empty()) {
    if (impl_->controller.getImuData(impl_->selected_imu_canu, imu)) {
      return true;
    }
    impl_->selected_imu_canu.clear();
  }

  for (const auto& canu : impl_->canus) {
    if (impl_->controller.getImuData(canu.name, imu)) {
      impl_->selected_imu_canu = canu.name;
      std::cout << "[BumiBackend] CAT IMU detected on CANU="
                << canu.name << std::endl;
      return true;
    }
  }

  imu.valid = false;
  return false;
}

bool BumiBackend::write(const std::vector<ActuatorCmd>& command) {
  if (!impl_ || !impl_->started || command.size() != impl_->order.size()) return false;

  const auto isAnkle = [&](std::size_t i) {
    for (const auto& a : impl_->ankles) if (i == a.pitch || i == a.roll) return true;
    return false;
  };

  {
    for (std::size_t i = 0; i < command.size(); ++i) {
      if (isAnkle(i)) continue;
      const auto& cmd = command[i];
      if (cmd.name != impl_->order[i]) {
        std::cerr << "[BumiBackend] Command order/name mismatch at " << i << std::endl;
        return false;
      }
      const auto& cal = impl_->calibration.at(cmd.name);
      impl_->controller.setMitCmd(cmd.name,
          (cmd.position - cal.bias) * cal.direction,
          cmd.velocity * cal.direction,
          cmd.torque_ff * cal.direction,
          cmd.kp, cmd.kd);
    }
    for (const auto& a : impl_->ankles) impl_->commandAnkle(a, command);
  }

  impl_->controller.stageCommand();
  impl_->controller.runCycle();
  return true;
}

bool BumiBackend::enableActuators() {
  if (!impl_ || !impl_->started) return false;
  if (impl_->controller.allActuatorsEnabled()) {
    impl_->enabled = true;
    return true;
  }
  for (int i = 0; i < kEnableConfirmCycles; ++i) {
    if (i % kEnableRetryCycles == 0 && !impl_->controller.enableAll()) {
      std::cerr << "[BumiBackend] EnableAllActuator request failed." << std::endl;
    }
    impl_->exchangeZero();
    if (impl_->controller.allActuatorsEnabled()) {
      impl_->enabled = true;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(impl_->cycle_ns));
  }
  impl_->enabled = false;
  std::cerr << "[BumiBackend] Timed out waiting for actuator enable." << std::endl;
  return false;
}

bool BumiBackend::disableActuators() {
  if (!impl_ || !impl_->configured) return false;

  // First remove commanded torque, then keep exchanging zero frames while
  // waiting for the vendor-reported enable state to become false. The disable
  // request is retried because drive state transitions are asynchronous.
  for (int i = 0; i < 10; ++i) impl_->exchangeZero();

  bool request_seen = false;
  for (int i = 0; i < kDisableConfirmCycles; ++i) {
    if (i % kDisableRetryCycles == 0) {
      const bool requested = impl_->controller.disableAll();
      request_seen = request_seen || requested;
      if (!requested) {
        std::cerr << "[BumiBackend] DisableAllActuator request failed; retrying."
                  << std::endl;
      }
    }

    impl_->exchangeZero();
    if (!impl_->controller.allActuatorsEnabled()) {
      impl_->enabled = false;
      return true;
    }

    std::this_thread::sleep_for(std::chrono::nanoseconds(impl_->cycle_ns));
  }

  impl_->enabled = impl_->controller.allActuatorsEnabled();
  std::cerr << "[BumiBackend] Timed out waiting for actuator disable; "
            << "request_seen=" << (request_seen ? "true" : "false")
            << " enabled=" << (impl_->enabled ? "true" : "false")
            << std::endl;
  return !impl_->enabled;
}

bool BumiBackend::actuatorsEnabled() const {
  return impl_ && impl_->enabled && impl_->controller.allActuatorsEnabled();
}

}  // namespace noetix
