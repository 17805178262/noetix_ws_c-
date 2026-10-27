#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "noetix_bridge/robot_resource.hpp"
#include "noetix_msgs/msg/low_cmd.hpp"
#include "noetix_msgs/msg/low_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace noetix_bridge {

class PoseModeNode final : public rclcpp::Node {
 public:
  // 构造共享姿态节点，统一处理使能姿态和站立。
  PoseModeNode()
  : rclcpp::Node("noetix_pose_mode_node")
  {
    declare_parameter<std::string>("robot_name", "n3");
    declare_parameter<bool>("trajectory_use_ros_time", false);

    robot_name_ = get_parameter("robot_name").as_string();
    trajectory_use_ros_time_ =
        get_parameter("trajectory_use_ros_time").as_bool();
    resource_ = loadRobotResource(robot_name_);

    state_sub_ = create_subscription<noetix_msgs::msg::LowState>(
        "/noetix/low_state", rclcpp::SensorDataQoS(),
        [this](noetix_msgs::msg::LowState::SharedPtr msg) {
          latest_state_ = std::move(msg);
          // 只有收到已使能反馈后才开始姿态，避免与底层异步使能发生竞态。
          if (enable_pending_ && latest_state_->status.mode ==
                  noetix_msgs::msg::RobotStatus::MODE_ENABLED) {
            enable_pending_ = false;
            startMode("enable");
          }
          if (active_ &&
              (latest_state_->status.mode == noetix_msgs::msg::RobotStatus::MODE_DISABLED ||
               latest_state_->status.mode == noetix_msgs::msg::RobotStatus::MODE_DAMPING)) {
            stopMode("drive disabled");
          }
        });

    request_sub_ = create_subscription<std_msgs::msg::String>(
        "/noetix/control_request", 20,
        [this](std_msgs::msg::String::SharedPtr msg) {
          onRequest(msg->data);
        });

    // A policy controller owns /noetix/low_cmd once it reports active=<name>.
    // Stop pose publishing immediately so only one upper layer sends commands.
    policy_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/noetix/policy_state", 20,
        [this](std_msgs::msg::String::SharedPtr msg) {
          onPolicyState(msg->data);
        });

    cmd_pub_ = create_publisher<noetix_msgs::msg::LowCmd>(
        "/noetix/low_cmd", 20);

    RCLCPP_INFO(
        get_logger(),
        "Pose-mode controller ready robot=%s enable=%s stand=%s",
        robot_name_.c_str(),
        resource_.enable_control.configured ? "configured" : "missing",
        resource_.stand_control.configured ? "configured" : "missing");
  }

 private:
  struct ActiveJointCommand {
    std::string actuator_name;
    double start_q{0.0};
    double target_q{0.0};
    double target_dq{0.0};
    double target_tau{0.0};
    double kp{0.0};
    double kd{0.0};
  };

  // 五次插值使姿态切换的起止速度连续。
  static double quintic(double t)
  {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
  }

  // 根据控制请求选择机器人本地姿态参数。
  const JointControlProfile* profileFor(const std::string& mode) const
  {
    if (mode == "enable") {
      return &resource_.enable_control;
    }
    if (mode == "stand") {
      return &resource_.stand_control;
    }
    return nullptr;
  }

  // 使能请求等待反馈后执行，停止请求同时取消未完成的使能姿态。
  void onRequest(std::string request)
  {
    std::transform(
        request.begin(), request.end(), request.begin(),
        [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });

    if (request == "disable" || request == "stop" ||
        request == "estop" || request == "safe_stop") {
      stopMode("stop request");
      return;
    }

    if (request == "enable") {
      stopMode("enable requested");
      enable_pending_ = true;
      RCLCPP_INFO(get_logger(), "Enable pose pending; waiting for enabled LowState");
      return;
    }
    if (request == "stand") {
      enable_pending_ = false;
      startMode(request);
    }
  }

  // 策略取得控制权后停止姿态命令，保持单一上层控制源。
  void onPolicyState(const std::string& text)
  {
    if (text.rfind("active=", 0) == 0 &&
        text.rfind("active=none", 0) != 0) {
      stopMode("policy took over");
    }
  }

  // 从当前关节反馈平滑过渡到配置姿态。
  void startMode(const std::string& mode)
  {
    const JointControlProfile* profile = profileFor(mode);
    if (profile == nullptr) {
      return;
    }

    if (!profile->configured) {
      RCLCPP_ERROR(get_logger(), "%s pose is not configured in %s",
                   mode.c_str(), resource_.robot_yaml.c_str());
      return;
    }

    if (!latest_state_) {
      RCLCPP_WARN(
          get_logger(),
          "%s request ignored: LowState is unavailable",
          mode.c_str());
      return;
    }

    if (latest_state_->status.mode ==
        noetix_msgs::msg::RobotStatus::MODE_DISABLED) {
      RCLCPP_WARN(
          get_logger(),
          "%s request ignored: robot is DISABLED",
          mode.c_str());
      return;
    }

    std::unordered_map<std::string, double> current_q;
    current_q.reserve(latest_state_->motor_state.size());
    for (const auto& motor : latest_state_->motor_state) {
      current_q.emplace(motor.name, static_cast<double>(motor.q));
    }

    if (profile->joints.size() != resource_.joints.size()) {
      RCLCPP_ERROR(
          get_logger(),
          "%s request ignored: profile DOF mismatch profile=%zu robot=%zu",
          mode.c_str(), profile->joints.size(), resource_.joints.size());
      return;
    }

    std::vector<ActiveJointCommand> next_commands;
    next_commands.reserve(resource_.joints.size());

    for (std::size_t i = 0; i < resource_.joints.size(); ++i) {
      const auto& mapping = resource_.joints[i];

      const auto state_it = current_q.find(mapping.actuator_name);
      if (state_it == current_q.end()) {
        RCLCPP_ERROR(
            get_logger(),
            "%s request ignored: actuator '%s' missing from LowState",
            mode.c_str(), mapping.actuator_name.c_str());
        return;
      }

      ActiveJointCommand command;
      command.actuator_name = mapping.actuator_name;
      command.start_q = state_it->second;

      const auto& source = profile->joints[i];
      if (source.joint_name != mapping.joint_name ||
          source.actuator_name != mapping.actuator_name) {
        RCLCPP_ERROR(get_logger(), "%s request ignored: profile order/mapping mismatch at index %zu",
                     mode.c_str(), i);
        return;
      }
      // robot.yaml使用逻辑关节坐标；下发前逆变换为执行器坐标。
      const double direction = static_cast<double>(mapping.direction);
      command.target_q = direction * (source.pos - mapping.offset);
      command.target_dq = direction * source.vel;
      command.target_tau = direction * source.ff;
      command.kp = source.kp;
      command.kd = source.kd;

      next_commands.push_back(std::move(command));
    }

    commands_ = std::move(next_commands);
    active_profile_ = profile;
    active_mode_ = mode;
    transition_complete_logged_ = false;
    sequence_ = 0;
    start_steady_ = std::chrono::steady_clock::now();
    start_ros_ = now();
    active_ = true;

    setPublishRate(profile->publish_hz);

    RCLCPP_INFO(
        get_logger(),
        "Mode -> %s robot=%s joints=%zu duration=%.3fs publish=%.1fHz source=%s",
        active_mode_.c_str(), robot_name_.c_str(), commands_.size(),
        profile->duration_s, profile->publish_hz,
        resource_.robot_yaml.c_str());
  }

  // 停止姿态定时器并取消等待中的使能姿态。
  void stopMode(const char* reason)
  {
    enable_pending_ = false;
    if (!active_) {
      return;
    }

    const std::string old_mode = active_mode_;
    active_ = false;
    active_profile_ = nullptr;
    active_mode_.clear();
    commands_.clear();

    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }

    RCLCPP_INFO(
        get_logger(),
        "Pose mode %s stopped: %s",
        old_mode.c_str(), reason);
  }

  // 按姿态档位配置发布定时器。
  void setPublishRate(double publish_hz)
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz));

    timer_ = create_wall_timer(period, [this]() { step(); });
  }

  // 按时间插值并持续发布目标，直到控制权交接。
  void step()
  {
    if (!active_ || active_profile_ == nullptr || commands_.empty()) {
      return;
    }

    const double elapsed = trajectory_use_ros_time_
        ? std::max(0.0, (now() - start_ros_).seconds())
        : std::chrono::duration<double>(
              std::chrono::steady_clock::now() - start_steady_).count();
    const double u = std::clamp(
        elapsed / active_profile_->duration_s, 0.0, 1.0);
    const double alpha = quintic(u);

    noetix_msgs::msg::LowCmd msg;
    msg.stamp = now();
    msg.sequence = ++sequence_;
    msg.motor_cmd.resize(commands_.size());

    for (std::size_t i = 0; i < commands_.size(); ++i) {
      const auto& source = commands_[i];
      auto& cmd = msg.motor_cmd[i];

      cmd.name = source.actuator_name;
      cmd.q = static_cast<float>(
          source.start_q + alpha * (source.target_q - source.start_q));
      cmd.dq = static_cast<float>(source.target_dq);
      cmd.tau = static_cast<float>(source.target_tau);
      cmd.kp = static_cast<float>(source.kp);
      cmd.kd = static_cast<float>(source.kd);
    }

    cmd_pub_->publish(msg);

    // 插值结束只表示目标指令已走完轨迹，不代表实测关节已经到位。
    // 持续发布最终目标，直到其他控制器接管或收到停止/失能请求。
    if (u >= 1.0 && !transition_complete_logged_) {
      transition_complete_logged_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Mode %s target interpolation finished; holding final command",
          active_mode_.c_str());
    }
  }

  RobotResourceInfo resource_;
  std::string robot_name_;

  bool active_{false};
  bool enable_pending_{false};
  bool transition_complete_logged_{false};
  std::uint64_t sequence_{0};
  std::string active_mode_;
  const JointControlProfile* active_profile_{nullptr};
  std::chrono::steady_clock::time_point start_steady_{};
  rclcpp::Time start_ros_{0, 0, RCL_ROS_TIME};
  bool trajectory_use_ros_time_{false};
  std::vector<ActiveJointCommand> commands_;
  noetix_msgs::msg::LowState::SharedPtr latest_state_;

  rclcpp::Subscription<noetix_msgs::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr policy_state_sub_;
  rclcpp::Publisher<noetix_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace noetix_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<noetix_bridge::PoseModeNode>());
  } catch (const std::exception& e) {
    std::cerr << "noetix_pose_mode_node fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
