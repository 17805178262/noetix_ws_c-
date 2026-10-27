// 标准库头文件
#include <algorithm>      // std::transform
#include <cctype>         // std::tolower
#include <chrono>         // 时间字面量，如 5ms
#include <cstdint>        // uint64_t
#include <iostream>       // std::cerr
#include <memory>         // std::shared_ptr, std::make_shared
#include <stdexcept>      // std::runtime_error
#include <string>         // std::string
#include <unordered_map>  // std::unordered_map
#include <utility>        // std::move

// 项目内部头文件
#include "noetix/config/robot_config.hpp"      // noetix::RobotConfig
#include "noetix/robot/robot_factory.hpp"      // noetix::CreateRobot
#include "noetix_bridge/io_loop.hpp"           // IoLoop
#include "noetix_bridge/robot_resource.hpp"    // RobotResourceInfo, loadRobotResource
#include "noetix_msgs/msg/low_cmd.hpp"         // 自定义 LowCmd 消息
#include "noetix_msgs/msg/low_state.hpp"       // 自定义 LowState 消息
#include "rclcpp/rclcpp.hpp"                   // ROS 2 C++ 客户端库
#include "sensor_msgs/msg/imu.hpp"             // 标准 IMU 消息
#include "sensor_msgs/msg/joint_state.hpp"     // 标准关节状态消息
#include "std_msgs/msg/string.hpp"             // 标准字符串消息

using namespace std::chrono_literals;  // 允许写 5ms, 2s 等

namespace noetix_bridge {

class NoetixBridgeNode final : public rclcpp::Node {
 public:
  NoetixBridgeNode() : rclcpp::Node("noetix_bridge_node")  // 节点名
  {
    // ---------- 1. 声明并读取参数 ----------
    declare_parameter<std::string>("robot_name", "n3");
    declare_parameter<double>("control_frequency_hz", 1000.0);
    declare_parameter<bool>("rt_realtime", true);
    declare_parameter<bool>("rt_lock_memory", true);
    declare_parameter<bool>("rt_strict", false);
    declare_parameter<int>("rt_priority", 80);
    declare_parameter<int>("rt_cpu", -1);

    control_frequency_hz_ = get_parameter("control_frequency_hz").as_double();

    // 实时线程选项
    rt_options_.realtime = get_parameter("rt_realtime").as_bool();
    rt_options_.lock_memory = get_parameter("rt_lock_memory").as_bool();
    rt_options_.strict_realtime = get_parameter("rt_strict").as_bool();
    rt_options_.priority = static_cast<int>(get_parameter("rt_priority").as_int());
    rt_options_.cpu_affinity = static_cast<int>(get_parameter("rt_cpu").as_int());

    robot_name_ = get_parameter("robot_name").as_string();

    // ---------- 2. 加载机器人资源 ----------
    // 从 robot_name 对应的配置目录加载自由度、执行器名称、关节映射等
    resource_ = loadRobotResource(robot_name_);

    // ---------- 3. 创建并配置硬件接口 ----------
    noetix::RobotConfig cfg;
    cfg.robot_name = robot_name_;
    cfg.config_root = resource_.robot_root;
    cfg.dof = resource_.dof;
    cfg.actuator_order = resource_.actuator_names;

    robot_hw_ = noetix::CreateRobot(cfg);
    if (!robot_hw_ || !robot_hw_->configure(cfg) || !robot_hw_->start()) {
      throw std::runtime_error("robot configure/start failed");
    }

    // ---------- 4. 创建 I/O 循环 ----------
    // IoLoop 负责以固定频率读写硬件，并管理命令与状态
    io_loop_ = std::make_shared<IoLoop>(robot_hw_, resource_.actuator_names);

    // ---------- 5. 创建 ROS 发布者 ----------
    // 底层状态：电机状态 + IMU 原始数据 + 机器人状态
    low_state_pub_ = create_publisher<noetix_msgs::msg::LowState>(
        "/noetix/low_state", 10);

    // 标准关节状态
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    // 标准 IMU
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/noetix/imu", 10);

    // 控制状态字符串（模式、周期、错误计数等）
    control_state_pub_ = create_publisher<std_msgs::msg::String>(
        "/noetix/control_state", 10);

    // ---------- 6. 创建 ROS 订阅者 ----------
    // 底层命令订阅
    low_cmd_sub_ = create_subscription<noetix_msgs::msg::LowCmd>(
        "/noetix/low_cmd", 20,
        [this](noetix_msgs::msg::LowCmd::SharedPtr msg) {
          onLowCmd(*msg);
        });

    // 控制请求订阅（enable/disable 等）
    request_sub_ = create_subscription<std_msgs::msg::String>(
        "/noetix/control_request", 20,
        [this](std_msgs::msg::String::SharedPtr msg) {
          onRequest(msg->data);
        });

    // ---------- 7. 创建定时器 ----------
    // 每 2ms 发布一次状态（约 500Hz，与控制频率 1000Hz 不同，是发布频率）
    state_timer_ = create_wall_timer(2ms, [this]() { publishState(); });
    // 每 2s 发布一次控制统计信息
    stats_timer_ = create_wall_timer(2s, [this]() { publishControlState(); });
    damping_timer_ = create_wall_timer(10ms, [this]() {
      if (io_loop_->dampingActive() && std::chrono::steady_clock::now() >= damping_end_) {
        if (io_loop_->disableDrive()) RCLCPP_INFO(get_logger(), "Damping finished; Mode -> DISABLED");
        else RCLCPP_ERROR(get_logger(), "DISABLE failed after damping");
      }
    });

    // ---------- 8. 启动 I/O 循环 ----------
    if (!io_loop_->start(control_frequency_hz_, rt_options_)) {
      throw std::runtime_error("I/O loop start failed");
    }

    RCLCPP_INFO(
        get_logger(),
        "Thin real-hardware bridge online robot=%s dof=%zu io=%.1fHz mode=DISABLED",
        robot_name_.c_str(), resource_.dof,
        control_frequency_hz_);
  }

  // 析构：停止 I/O 循环和硬件
  ~NoetixBridgeNode() override
  {
    if (io_loop_) {
      io_loop_->stop();
    }
    if (robot_hw_) {
      robot_hw_->stop();
    }
  }

 private:
  // ---------- 处理底层命令 ----------
  void onLowCmd(const noetix_msgs::msg::LowCmd& msg)
  {
    ++low_cmd_received_;

    // 将 ROS 消息转换为内部 noetix::LowCmd
    noetix::LowCmd cmd;
    cmd.sequence = msg.sequence;
    cmd.motor_cmd.reserve(msg.motor_cmd.size());

    for (const auto& src : msg.motor_cmd) {
      noetix::MotorCmd dst;
      dst.name = src.name;
      dst.q = src.q;
      dst.dq = src.dq;
      dst.tau = src.tau;
      dst.kp = src.kp;
      dst.kd = src.kd;
      cmd.motor_cmd.push_back(std::move(dst));
    }

    // 提交给 I/O 循环；如果驱动器未使能、执行器不匹配或值非有限，则拒绝
    if (!io_loop_->setCommand(cmd)) {
      ++low_cmd_rejected_;
      if (!command_rejection_logged_) {
        RCLCPP_WARN(
          get_logger(),
          "LowCmd ignored: drive disabled, command must contain exactly %zu unique configured actuators, or a value is non-finite",
          resource_.dof);
        command_rejection_logged_ = true;
      }
    } else {
      ++low_cmd_accepted_;
      command_rejection_logged_ = false;
    }
  }

  // ---------- 处理控制请求 ----------
  void onRequest(std::string request)
  {
    // 转小写，便于比较
    std::transform(
        request.begin(), request.end(), request.begin(),
        [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });

    if (request == "enable") {
      if (io_loop_->driveEnabled() && !io_loop_->dampingActive()) {
        return;  // 已经使能，直接返回
      }
      if (!io_loop_->enableDrive()) {
        RCLCPP_ERROR(get_logger(), "ENABLE failed");
        return;
      }
      RCLCPP_INFO(get_logger(), "Mode -> ENABLED");
      return;
    }

    // 普通失能先阻尼三秒，重复请求不延长时间；不触发清错。
    if (request == "disable") {
      if (!io_loop_->driveEnabled() || io_loop_->dampingActive()) return;
      if (io_loop_->beginDamping(8.0)) {
        damping_end_ = std::chrono::steady_clock::now() + 3s;
        RCLCPP_INFO(get_logger(), "Mode -> DAMPING; kp=0 kd=8; DISABLE in 3s");
      } else {
        io_loop_->disableDrive();
        RCLCPP_ERROR(get_logger(), "Damping command failed; requested immediate DISABLE");
      }
      return;
    }
    // 人工清错只在完全失能时执行，不主动失能或重新使能。
    if (request == "clear_errors") {
      if (io_loop_->driveEnabled()) {
        RCLCPP_WARN(get_logger(), "Motor error clear ignored: wait for fully DISABLED state");
      } else if (io_loop_->clearMotorErrors()) {
        RCLCPP_INFO(get_logger(), "Motor error clear request sent");
      } else {
        RCLCPP_WARN(get_logger(), "Motor error clear failed or is unsupported for robot=%s", robot_name_.c_str());
      }
      return;
    }
    // 急停始终立即关闭驱动，不等待阻尼阶段。
    if (request == "stop" ||
        request == "estop" || request == "safe_stop") {
      const bool was_enabled = io_loop_->driveEnabled();
      if (!io_loop_->disableDrive()) {
        RCLCPP_ERROR(get_logger(), "DISABLE failed");
      } else if (was_enabled) {
        RCLCPP_INFO(get_logger(), "Mode -> DISABLED");
      }
      return;
    }

    // stand 是上层姿态模式，由 noetix_pose_mode_node 处理，
    // 本节点只负责接收请求但不直接执行，因为上层会发布普通 LowCmd。
    if (request == "stand") {
      return;
    }

    RCLCPP_WARN(
        get_logger(),
        "Unknown control request: %s (valid: enable, stand, disable, estop, clear_errors)",
        request.c_str());
  }

  // ---------- 发布状态 ----------
  void publishState()
  {
    noetix::LowState state;
    if (!io_loop_->getLatestState(state)) {
      return;  // 暂无新状态
    }

    const auto stamp = now();

    // 构造自定义 LowState 消息
    noetix_msgs::msg::LowState low_state;
    low_state.stamp = stamp;
    low_state.sequence = state.sequence;
    low_state.motor_state.resize(state.motor_state.size());

    for (std::size_t i = 0; i < state.motor_state.size(); ++i) {
      auto& dst = low_state.motor_state[i];
      const auto& src = state.motor_state[i];
      dst.name = src.name;
      dst.q = src.q;
      dst.dq = src.dq;
      dst.tau_est = src.tau_est;
      dst.temperature = src.temperature;
      dst.error = src.error;
    }

    // IMU 原始数据
    low_state.imu.valid = state.imu.valid;
    last_imu_valid_ = state.imu.valid;
    for (int i = 0; i < 4; ++i) {
      low_state.imu.quaternion[i] = state.imu.quaternion[i];
    }
    for (int i = 0; i < 3; ++i) {
      low_state.imu.gyroscope[i] = state.imu.gyroscope[i];
      low_state.imu.accelerometer[i] = state.imu.accelerometer[i];
    }

    // 机器人状态：模式（使能/禁用）、连接、故障
    low_state.status.mode = io_loop_->dampingActive()
        ? noetix_msgs::msg::RobotStatus::MODE_DAMPING
        : io_loop_->driveEnabled() ? noetix_msgs::msg::RobotStatus::MODE_ENABLED
                                  : noetix_msgs::msg::RobotStatus::MODE_DISABLED;
    low_state.status.connected = state.status.connected;
    low_state.status.enabled = io_loop_->driveEnabled();
    low_state.status.fault = state.status.fault;
    low_state_pub_->publish(low_state);

    // 构造标准 JointState
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name = resource_.joint_names;
    joint_state.position.assign(resource_.dof, 0.0);
    joint_state.velocity.assign(resource_.dof, 0.0);
    joint_state.effort.assign(resource_.dof, 0.0);

    // 建立执行器名称到电机状态的映射，便于查找
    std::unordered_map<std::string, const noetix::MotorState*> by_actuator;
    by_actuator.reserve(state.motor_state.size());
    for (const auto& motor : state.motor_state) {
      by_actuator.emplace(motor.name, &motor);
    }

    // 根据关节映射，将电机状态转换为关节状态
    for (std::size_t i = 0; i < resource_.joints.size(); ++i) {
      const auto& mapping = resource_.joints[i];
      const auto it = by_actuator.find(mapping.actuator_name);
      if (it == by_actuator.end()) {
        continue;  // 找不到对应执行器，跳过
      }
      const auto& motor = *it->second;
      const double direction = static_cast<double>(mapping.direction);
      joint_state.position[i] = direction * motor.q + mapping.offset;
      joint_state.velocity[i] = direction * motor.dq;
      joint_state.effort[i] = direction * motor.tau_est;
    }
    joint_state_pub_->publish(joint_state);

    // 如果 IMU 数据有效，发布标准 Imu 消息
    if (state.imu.valid) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = resource_.imu_frame;
      imu.orientation.w = state.imu.quaternion[0];
      imu.orientation.x = state.imu.quaternion[1];
      imu.orientation.y = state.imu.quaternion[2];
      imu.orientation.z = state.imu.quaternion[3];
      imu.angular_velocity.x = state.imu.gyroscope[0];
      imu.angular_velocity.y = state.imu.gyroscope[1];
      imu.angular_velocity.z = state.imu.gyroscope[2];
      imu.linear_acceleration.x = state.imu.accelerometer[0];
      imu.linear_acceleration.y = state.imu.accelerometer[1];
      imu.linear_acceleration.z = state.imu.accelerometer[2];
      imu_pub_->publish(imu);
    }
  }

  // ---------- 发布控制统计信息 ----------
  void publishControlState()
  {
    const auto rt = io_loop_->stats();  // 获取 I/O 循环统计
    std_msgs::msg::String msg;
    msg.data =
        std::string("mode=") +
        (io_loop_->dampingActive() ? "DAMPING" : io_loop_->driveEnabled() ? "ENABLED" : "DISABLED") +
        " command=" + (io_loop_->hasCommand() ? "present" : "none") +
        " cycles=" + std::to_string(rt.cycles) +
        " overruns=" + std::to_string(rt.overruns) +
        " read_errors=" + std::to_string(io_loop_->readErrors()) +
        " write_errors=" + std::to_string(io_loop_->writeErrors()) +
        " imu=" + (last_imu_valid_ ? "valid" : "invalid") +
        " cmd_rx=" + std::to_string(low_cmd_received_) +
        " cmd_ok=" + std::to_string(low_cmd_accepted_) +
        " cmd_reject=" + std::to_string(low_cmd_rejected_);
    control_state_pub_->publish(msg);
    RCLCPP_DEBUG(get_logger(), "%s", msg.data.c_str());
    const auto reads = io_loop_->readErrors();
    const auto writes = io_loop_->writeErrors();
    const bool io_errors = reads > last_read_errors_ || writes > last_write_errors_;
    if (io_errors && !io_errors_logged_) {
      RCLCPP_WARN(get_logger(), "Hardware I/O errors: read=%llu write=%llu",
                  static_cast<unsigned long long>(reads), static_cast<unsigned long long>(writes));
    } else if (!io_errors && io_errors_logged_) {
      RCLCPP_INFO(get_logger(), "Hardware I/O recovered");
    }
    io_errors_logged_ = io_errors;
    last_read_errors_ = reads;
    last_write_errors_ = writes;
    if (!imu_status_logged_ || last_imu_valid_ != reported_imu_valid_) {
      if (last_imu_valid_) {
        RCLCPP_INFO(get_logger(), "IMU ready");
      } else {
        RCLCPP_WARN(get_logger(), "IMU invalid; waiting for valid feedback");
      }
      reported_imu_valid_ = last_imu_valid_;
      imu_status_logged_ = true;
    }
  }

  // ---------- 成员变量 ----------
  rclcpp::TimerBase::SharedPtr damping_timer_;
  std::chrono::steady_clock::time_point damping_end_{};
  RobotResourceInfo resource_;                 // 机器人资源信息
  std::string robot_name_;                     // 机器人名称
  double control_frequency_hz_{1000.0};        // 控制频率
  noetix::PeriodicThreadOptions rt_options_{}; // 实时线程选项

  std::shared_ptr<noetix::RobotInterface> robot_hw_;  // 硬件接口
  std::shared_ptr<IoLoop> io_loop_;                   // I/O 循环

  // 发布者
  rclcpp::Publisher<noetix_msgs::msg::LowState>::SharedPtr low_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_pub_;

  // 订阅者
  rclcpp::Subscription<noetix_msgs::msg::LowCmd>::SharedPtr low_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // 状态变量
  bool last_imu_valid_{false};
  bool reported_imu_valid_{false};
  bool imu_status_logged_{false};
  bool command_rejection_logged_{false};
  bool io_errors_logged_{false};
  std::uint64_t last_read_errors_{0};
  std::uint64_t last_write_errors_{0};
  std::uint64_t low_cmd_received_{0};
  std::uint64_t low_cmd_accepted_{0};
  std::uint64_t low_cmd_rejected_{0};
};

}  // namespace noetix_bridge

// ---------- 主函数 ----------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  // 初始化 ROS 2
  try {
    // 创建节点并进入自旋（处理回调）
    // std::make_shared<noetix_bridge::NoetixBridgeNode>() 创建一个节点对象，这一步立即执行构造函数，会加载配置、启动硬件和创建 I/O 线程。
    // rclcpp::spin(...) 让 ROS 2 不断处理订阅消息和定时器回调。否则虽然对象创建了，但订阅回调不会持续执行。
    rclcpp::spin(std::make_shared<noetix_bridge::NoetixBridgeNode>());
  } catch (const std::exception& e) {
    std::cerr << "noetix_bridge_node fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
