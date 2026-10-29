// 中文说明：ROS2 C++适配层，保留独立策略订阅、统一手柄切换和LowCmd话题契约。
// GMT真机使用独立单调时钟线程；仿真使用ROS时钟。控制状态、传感器快照和参考缓冲分别加锁。
// stop与推理互斥，join在执行锁外；切换GMT先准备真实首帧，再停旧策略并用最新反馈重新计算。
#pragma once
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <noetix_msgs/msg/low_cmd.hpp>
#include <noetix_msgs/msg/low_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "noetix_policy_cpp/inference.hpp"
#include "noetix_policy_cpp/resources.hpp"
namespace noetix::policy {
double steady_seconds();
class PolicyNode : public rclcpp::Node {
 public:
  PolicyNode(const std::string& name, const rclcpp::NodeOptions& options, bool pending = false);
  ~PolicyNode() override;
  void start();
  void shutdown(bool release = true);
  void stop(const std::string& reason, bool release = false);
  bool commit_handoff();
  bool running() const { return running_.load(); }
  bool ready() const { return ready_.load(); }

 private:
  PolicyNode(Spec definition, const rclcpp::NodeOptions& options, bool pending);
  struct Mapping {
    std::string actuator;
    double direction = 1, offset = 0;
  };
  PolicyResources resources_;
  std::unique_ptr<IPolicy> policy_;
  Inference inference_;
  std::vector<Mapping> mapping_;
  std::vector<std::string> cached_names_;
  std::vector<size_t> indices_;
  std::array<int, 3> axes_{};
  F3 limits_ = F3::Zero(), signs_ = F3::Ones();
  std::string joy_topic_, odom_topic_, reference_status_;
  std::recursive_mutex execution_mutex_;
  std::mutex data_mutex_, reference_mutex_, wait_mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  std::atomic<bool> running_{true}, ready_{false};
  bool pending_ = false, announced_ = false, first_ready_ = false, simulation_ = false;
  uint64_t sequence_ = 0;
  noetix_msgs::msg::LowState::ConstSharedPtr state_;
  sensor_msgs::msg::Joy::ConstSharedPtr joy_;
  std::optional<V3> position_;
  double position_received_ = 0, warning_at_ = -1e9;
  noetix_msgs::msg::LowCmd command_;
  std::vector<double> reference_packet_;
  std::array<std::array<double, 5>, 500> timings_{};
  size_t timing_size_ = 0;
  std::array<double, 4> last_timing_{};
  double report_at_ = 0;
  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::Publisher<noetix_msgs::msg::LowCmd>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<noetix_msgs::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void tick();
  void step(const noetix_msgs::msg::LowState& state);
  State read_state(const noetix_msgs::msg::LowState& state);
  void fill_command(const float* target, bool zero = false);
  void announce(const std::string& value);
  void report_reference();
  void record_timing();
  void receive_reference(const std_msgs::msg::Float32MultiArray& message);
};
class PolicySwitch : public rclcpp::Node {
 public:
  PolicySwitch(std::shared_ptr<rclcpp::Executor> executor,
               const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PolicySwitch() override;
  void start();
  void shutdown();
  bool has_retired() const { return retired_ready_.load(); }
  // 必须在所有执行器工作线程join且旧Executor/等待集销毁之后调用。
  void collect_retired();
  void detach_executor();
  void attach_executor(std::shared_ptr<rclcpp::Executor> executor);

 private:
  struct Binding {
    std::vector<int> buttons;
    std::string target;
    bool control = false;
  };
  std::shared_ptr<rclcpp::Executor> executor_;
  std::recursive_mutex mutex_;
  std::vector<Binding> bindings_;
  std::vector<int32_t> previous_;
  std::string config_, joy_topic_;
  bool closing_ = false;
  uint64_t generation_ = 0;
  std::shared_ptr<PolicyNode> current_, pending_;
  std::vector<std::shared_ptr<PolicyNode>> retired_;
  std::atomic<bool> retired_ready_{false};
  rclcpp::CallbackGroup::SharedPtr joy_group_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr handoff_timer_;
  void joy(const sensor_msgs::msg::Joy& message);
  void select(const std::string& name);
  void control(const std::string& action);
  void finish_handoff();
  void dispose(std::shared_ptr<PolicyNode>& node, bool release);
  void clear(bool release);
};
}  // namespace noetix::policy
