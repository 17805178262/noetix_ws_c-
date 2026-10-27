// 中文说明：独立策略ROS节点，保持原LowState/LowCmd、手柄、GMT和可选里程计接口。
// 传感器回调只替换共享消息快照；执行器名称索引仅在布局变化时重建，但每帧仍检查名称与数值。
// 预分配LowCmd和输入张量，GMT控制线程不等待ROS定时器队列；取消时不追补旧周期、不再补发命令。
// 日志每5秒给出实际发布频率、周期尾延迟、观测/推理/整步耗时及反馈年龄，区分配置与实测。
#include "noetix_policy_cpp/ros_policy.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <noetix_msgs/msg/robot_status.hpp>
#include <rclcpp/create_timer.hpp>
#include <set>
#include <sstream>
#include <unordered_map>
namespace noetix::policy {
namespace {
std::string share() { return ament_index_cpp::get_package_share_directory("noetix_policy"); }
std::string cleaned(std::string value) {
  auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
    return {};
  value = value.substr(begin, value.find_last_not_of(" \t\r\n") - begin + 1);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return value;
}
bool stop_request(const std::string& value) {
  return value == "disable" || value == "stop" || value == "estop" || value == "safe_stop" ||
         value == "enable" || value == "stand";
}
double quantile(std::vector<double> data, double p) {
  std::sort(data.begin(), data.end());
  double x = (data.size() - 1) * p;
  size_t i = size_t(x);
  return data[i] + (data[std::min(i + 1, data.size() - 1)] - data[i]) * (x - i);
}
}  // namespace
double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// 从注册名称获得训练契约，资源配置只在此次构造时读取。
PolicyNode::PolicyNode(const std::string& name, const rclcpp::NodeOptions& options, bool pending)
    : PolicyNode(policy_spec(name), options, pending) {}
PolicyNode::PolicyNode(Spec definition, const rclcpp::NodeOptions& options, bool pending)
    : Node(definition.name + "_policy", options),
      resources_(load_resources(declare_parameter<std::string>(
          "joystick_config", share() + "/config/joystick.yaml"), definition.name,
          definition.kind == Kind::motion)),
      policy_(make_policy(std::move(definition), resources_.motion)),
      inference_(policy_->config, policy_->inputs, load_model(policy_->config, resources_.model)),
      pending_(pending) {
  const auto kind = policy_->config.kind;
  auto path = ament_index_cpp::get_package_share_directory("noetix_robots") + "/" +
              policy_->config.robot + "/config/robot.yaml";
  auto resources = YAML::LoadFile(path);
  std::map<std::string, YAML::Node> mappings;
  for (auto item : resources["joints"])
    mappings[item["name"].as<std::string>()] = static_cast<const YAML::Node&>(item);
  for (auto& name : policy_->config.names) {
    require(mappings.count(name), "Missing robot mapping: " + name);
    auto item = mappings.at(name);
    Mapping map{item["actuator"].as<std::string>(), item["direction"].as<double>(),
                item["offset"].as<double>()};
    require((map.direction == 1 || map.direction == -1) && std::isfinite(map.offset),
            "Invalid mapping direction/offset");
    mapping_.push_back(map);
  }
  auto joystick = get_parameter("joystick_config").as_string();
  auto config = YAML::LoadFile(joystick)["joystick"];
  joy_topic_ = config["topic"].as<std::string>();
  int i = 0;
  for (auto key : {"x", "y", "yaw"}) {
    axes_[i] = config["axes"][key].as<int>();
    limits_[i] = config["limits"][key].as<float>();
    signs_[i] = config["signs"][key].as<float>();
    require(axes_[i] >= 0 && std::isfinite(limits_[i]) && std::isfinite(signs_[i]),
            "Invalid joystick axis config");
    ++i;
  }
  odom_topic_ = declare_parameter<std::string>("odometry_topic", "");
  command_.motor_cmd.resize(policy_->config.dof());
  for (int j = 0; j < policy_->config.dof(); ++j)
    command_.motor_cmd[j].name = mapping_[j].actuator;
  reference_packet_.resize(14 + policy_->config.dof());
  indices_.resize(policy_->config.dof());
  report_at_ = steady_seconds();
  command_pub_ = create_publisher<noetix_msgs::msg::LowCmd>("/noetix/low_cmd", 1);
  status_pub_ = create_publisher<std_msgs::msg::String>("/noetix/policy_state", 10);
  RCLCPP_INFO(get_logger(), "C++ %s model: %s; sha256=%s; period=20ms", policy_->config.name.c_str(),
              resources_.model.c_str(), inference_.model->sha256.c_str());
  if (!resources_.motion.empty())
    RCLCPP_INFO(get_logger(), "Motion resource: %s", resources_.motion.c_str());
  RCLCPP_INFO(get_logger(), "C++ inference: backend=%s; selected_threads=%d; ORT=%s; %s",
              inference_.model->backend.c_str(), inference_.model->threads,
              OrtGetApiBase()->GetVersionString(), inference_.model->report.c_str());
  if (kind == Kind::gmt)
    RCLCPP_INFO(get_logger(), "GMT ankle gains: pitch kp/kd=80/4, roll kp/kd=50/2; knee=200/15");
}
void PolicyNode::start() {
  auto weak = std::weak_ptr<PolicyNode>(std::static_pointer_cast<PolicyNode>(shared_from_this()));
  input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input;
  input.callback_group = input_group_;
  auto qos = rclcpp::SensorDataQoS();
  if (policy_->config.kind == Kind::gmt)
    qos.keep_last(1);
  state_sub_ = create_subscription<noetix_msgs::msg::LowState>(
      "/noetix/low_state", qos,
      [weak](noetix_msgs::msg::LowState::ConstSharedPtr value) {
        if (auto self = weak.lock()) {
          auto mode = value->status.mode;
          {
            std::lock_guard<std::mutex> lock(self->data_mutex_);
            self->state_ = std::move(value);
          }
          if (mode == noetix_msgs::msg::RobotStatus::MODE_DISABLED ||
              mode == noetix_msgs::msg::RobotStatus::MODE_DAMPING)
            self->stop("Drive disabled");
        }
      },
      input);
  request_sub_ = create_subscription<std_msgs::msg::String>(
      "/noetix/control_request", 20, [weak](std_msgs::msg::String::ConstSharedPtr value) {
        if (auto self = weak.lock())
          if (stop_request(cleaned(value->data)))
            self->stop("Control request: " + value->data);
      });
  if (policy_->config.kind == Kind::walk)
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::SensorDataQoS(),
        [weak](sensor_msgs::msg::Joy::ConstSharedPtr value) {
          if (auto self = weak.lock()) {
            std::lock_guard<std::mutex> lock(self->data_mutex_);
            self->joy_ = std::move(value);
          }
        },
        input);
  if (policy_->config.kind == Kind::gmt) {
    reference_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/noetix/gmt", 1,
        [weak](std_msgs::msg::Float32MultiArray::ConstSharedPtr value) {
          if (auto self = weak.lock())
            self->receive_reference(*value);
        },
        input);
    if (!odom_topic_.empty())
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, rclcpp::SensorDataQoS(),
          [weak](nav_msgs::msg::Odometry::ConstSharedPtr value) {
            if (auto self = weak.lock()) {
              auto& p = value->pose.pose.position;
              V3 position(p.x, p.y, p.z);
              if (position.allFinite()) {
                std::lock_guard<std::mutex> lock(self->data_mutex_);
                self->position_ = position;
                self->position_received_ = steady_seconds();
              }
            }
          },
          input);
  }
  bool simulation = get_parameter("use_sim_time").as_bool();
  if (policy_->config.kind == Kind::gmt && !simulation) {
    worker_ = std::thread([this] {
      auto period = std::chrono::milliseconds(20);
      auto deadline = std::chrono::steady_clock::now() + period;
      while (running_) {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        if (wake_.wait_until(lock, deadline, [this] { return !running_; }))
          break;
        lock.unlock();
        auto started = std::chrono::steady_clock::now();
        tick();
        deadline = started - deadline >= std::chrono::milliseconds(5) ? started + period
                                                                      : deadline + period;
      }
    });
    RCLCPP_INFO(get_logger(), "GMT scheduler: monotonic control thread");
  } else {
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(.02), [weak] {
      if (auto self = weak.lock())
        self->tick();
    });
  }
}
State PolicyNode::read_state(const noetix_msgs::msg::LowState& value) {
  bool changed = cached_names_.size() != value.motor_state.size();
  if (!changed)
    for (size_t i = 0; i < cached_names_.size(); ++i)
      if (cached_names_[i] != value.motor_state[i].name) {
        changed = true;
        break;
      }
  if (changed) {
    std::unordered_map<std::string, size_t> names;
    std::vector<std::string> layout;
    for (size_t i = 0; i < value.motor_state.size(); ++i) {
      auto& name = value.motor_state[i].name;
      require(names.emplace(name, i).second, "LowState contains duplicate actuators");
      layout.push_back(name);
    }
    for (int j = 0; j < policy_->config.dof(); ++j) {
      require(names.count(mapping_[j].actuator), "Missing actuator: " + mapping_[j].actuator);
      indices_[j] = names.at(mapping_[j].actuator);
    }
    cached_names_ = std::move(layout);
  }
  require(!cached_names_.empty() && value.imu.valid, "IMU invalid or no feedback");
  State state;
  for (int j = 0; j < policy_->config.dof(); ++j) {
    auto& motor = value.motor_state[indices_[j]];
    auto& map = mapping_[j];
    state.q[j] = float(map.direction * double(motor.q) + map.offset);
    state.dq[j] = float(map.direction * double(motor.dq));
    require(std::isfinite(state.q[j]) && std::isfinite(state.dq[j]), "Feedback contains NaN/Inf");
  }
  for (int j = 0; j < 4; ++j)
    state.quat[j] = value.imu.quaternion[j];
  for (int j = 0; j < 3; ++j)
    state.gyro[j] = value.imu.gyroscope[j];
  require(state.gyro.allFinite(), "Gyroscope contains NaN/Inf");
  state.quat = normalize(state.quat);
  state.gravity = rotate(conjugate(state.quat), F3(0, 0, -1));
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (position_ && steady_seconds() - position_received_ <= .2)
      state.position = position_;
  }
  return state;
}
void PolicyNode::fill_command(const float* target, bool zero) {
  command_.stamp = now();
  command_.sequence = sequence_;
  for (int j = 0; j < policy_->config.dof(); ++j) {
    auto& motor = command_.motor_cmd[j];
    auto& map = mapping_[j];
    require(std::isfinite(target[j]), "Invalid target");
    motor.q = float(map.direction * (double(target[j]) - map.offset));
    motor.dq = motor.tau = 0;
    motor.kp = zero ? 0 : policy_->config.kp[j];
    motor.kd = zero ? 0 : policy_->config.kd[j];
  }
}
void PolicyNode::step(const noetix_msgs::msg::LowState& value) {
  auto stamp = now();
  double started = steady_seconds();
  auto state = read_state(value);
  F3 command = F3::Zero();
  if (policy_->config.kind == Kind::walk) {
    sensor_msgs::msg::Joy::ConstSharedPtr joy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      joy = joy_;
    }
    for (int j = 0; j < 3; ++j) {
      float x = joy && size_t(axes_[j]) < joy->axes.size() ? joy->axes[axes_[j]] : 0;
      require(std::isfinite(x), "Joystick contains NaN/Inf");
      command[j] = std::clamp(x, -1.f, 1.f) * limits_[j] * signs_[j];
    }
  }
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    policy_->build(state, command, steady_seconds());
  }
  double built = steady_seconds();
  auto* action = inference_.run();
  double inferred = steady_seconds();
  policy_->decode(action, policy_->config.dof());
  fill_command(policy_->target.data());
  command_.stamp = stamp;
  last_timing_ = {(built - started) * 1000, (inferred - built) * 1000,
                  (steady_seconds() - started) * 1000,
                  std::max(0., (stamp.seconds() - rclcpp::Time(value.stamp).seconds()) * 1000)};
}
void PolicyNode::announce(const std::string& value) {
  std_msgs::msg::String msg;
  msg.data = value;
  status_pub_->publish(msg);
}
void PolicyNode::tick() {
  std::lock_guard<std::recursive_mutex> lock(execution_mutex_);
  if (!running_)
    return;
  noetix_msgs::msg::LowState::ConstSharedPtr state;
  {
    std::lock_guard<std::mutex> data(data_mutex_);
    state = state_;
  }
  if (!state || state->status.mode != noetix_msgs::msg::RobotStatus::MODE_ENABLED ||
      (pending_ && ready_))
    return;
  try {
    if (!announced_) {
      auto initial = read_state(*state);
      {
        std::lock_guard<std::mutex> ref(reference_mutex_);
        policy_->reset();
        if (auto* ref = policy_->reference())
          ref->align(initial.quat.cast<double>(), initial.position);
      }
      if (!pending_)
        announce("active=" + policy_->config.name);
      announced_ = true;
    }
    ++sequence_;
    step(*state);
    if (pending_) {
      first_ready_ = true;
      ready_ = true;
      return;
    }
    command_pub_->publish(command_);
    if (policy_->config.kind == Kind::gmt) {
      record_timing();
      report_reference();
    }
  } catch (const ReferenceNotReady&) {
    if (policy_->config.kind == Kind::gmt)
      report_reference();
  } catch (const std::exception& e) {
    stop(std::string("Policy error: ") + e.what(), true);
  }
}
bool PolicyNode::commit_handoff() {
  std::lock_guard<std::recursive_mutex> lock(execution_mutex_);
  if (!running_ || !first_ready_)
    return false;
  noetix_msgs::msg::LowState::ConstSharedPtr state;
  {
    std::lock_guard<std::mutex> data(data_mutex_);
    state = state_;
  }
  if (!state || state->status.mode != noetix_msgs::msg::RobotStatus::MODE_ENABLED)
    return false;
  policy_->reset_history();
  try {
    ++sequence_;
    step(*state);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "GMT handoff failed: %s", e.what());
    return false;
  }
  pending_ = false;
  first_ready_ = false;
  command_pub_->publish(command_);
  record_timing();
  announce("active=gmt");
  report_reference();
  return true;
}
void PolicyNode::stop(const std::string& reason, bool release) {
  std::lock_guard<std::recursive_mutex> lock(execution_mutex_);
  if (!running_.exchange(false))
    return;
  if (timer_)
    timer_->cancel();
  wake_.notify_all();
  noetix_msgs::msg::LowState::ConstSharedPtr state;
  {
    std::lock_guard<std::mutex> data(data_mutex_);
    state = state_;
  }
  if (release && !pending_ && rclcpp::ok(get_node_base_interface()->get_context()) && state &&
      state->status.mode == noetix_msgs::msg::RobotStatus::MODE_ENABLED) {
    ++sequence_;
    fill_command(policy_->config.default_q.data(), true);
    command_pub_->publish(command_);
  }
  if (announced_ && !pending_ && rclcpp::ok(get_node_base_interface()->get_context()))
    announce("active=none reason=" + reason);
  RCLCPP_INFO(get_logger(), "Policy stopped: %s", reason.c_str());
}
void PolicyNode::shutdown(bool release) {
  stop("Policy node destroyed", release);
  if (worker_.joinable())
    worker_.join();
}
PolicyNode::~PolicyNode() { shutdown(true); }
void PolicyNode::receive_reference(const std_msgs::msg::Float32MultiArray& message) {
  std::lock_guard<std::mutex> lock(reference_mutex_);
  if (!running_)
    return;
  auto* reference = policy_->reference();
  require(reference != nullptr, "Reference input only supported for GMT");
  std::string reason;
  const auto& dims = message.layout.dim;
  size_t width = reference_packet_.size();
  if (dims.size() != 1 || dims[0].label != "n3" || dims[0].size != width ||
      dims[0].stride != width || message.layout.data_offset != 0 || message.data.size() != width) {
    ++reference->rejected;
    reference->last_rejection = "GMT layout must contain 43 n3 values";
    reason = reference->last_rejection;
  } else {
    std::copy(message.data.begin(), message.data.end(), reference_packet_.begin());
    auto rejected = reference->rejected;
    reference->push(reference_packet_, steady_seconds());
    if (reference->rejected > rejected)
      reason = reference->last_rejection;
  }
  if (!reason.empty() && steady_seconds() - warning_at_ >= 5) {
    warning_at_ = steady_seconds();
    RCLCPP_WARN(get_logger(), "Ignoring invalid GMT frame: %s", reason.c_str());
  }
}
void PolicyNode::report_reference() {
  std::lock_guard<std::mutex> lock(reference_mutex_);
  const auto* reference = policy_->reference();
  if (!reference || reference_status_ == reference->status)
    return;
  reference_status_ = reference->status;
  if (reference_status_ == "holding" || reference_status_ == "refilling")
    RCLCPP_WARN(get_logger(), "GMT reference held; status=%s", reference_status_.c_str());
  else
    RCLCPP_INFO(get_logger(), "GMT reference status=%s; %s", reference_status_.c_str(),
                reference->last_segment_reason.c_str());
}
void PolicyNode::record_timing() {
  double at = steady_seconds();
  if (timing_size_ == timings_.size()) {
    std::move(timings_.begin() + 1, timings_.end(), timings_.begin());
    --timing_size_;
  }
  auto& row = timings_[timing_size_++];
  row[0] = at;
  std::copy(last_timing_.begin(), last_timing_.end(), row.begin() + 1);
  if (at - report_at_ < 5 || timing_size_ < 2)
    return;
  std::array<double, 4> mean{}, p95{};
  std::vector<double> periods;
  size_t over = 0;
  for (size_t j = 0; j < 4; ++j) {
    std::vector<double> values;
    values.reserve(timing_size_);
    for (size_t i = 0; i < timing_size_; ++i) {
      double x = timings_[i][j + 1];
      mean[j] += x;
      values.push_back(x);
    }
    mean[j] /= timing_size_;
    p95[j] = quantile(values, .95);
  }
  for (size_t i = 0; i < timing_size_; ++i) {
    over += timings_[i][3] > 20;
    if (i)
      periods.push_back((timings_[i][0] - timings_[i - 1][0]) * 1000);
  }
  double hz = (timing_size_ - 1) / (at - timings_[0][0]), period95 = quantile(periods, .95),
         maximum = *std::max_element(periods.begin(), periods.end());
  std::ostringstream text;
  text << "GMT timing: publish=" << hz << " Hz, target=50 Hz; period_p95/max=" << period95 << "/"
       << maximum << " ms; observation_mean/p95=" << mean[0] << "/" << p95[0]
       << " ms, inference_mean/p95=" << mean[1] << "/" << p95[1] << " ms, step_mean/p95=" << mean[2]
       << "/" << p95[2] << " ms, step_over20ms=" << 100. * over / timing_size_
       << "%, feedback_age_mean/p95=" << mean[3] << "/" << p95[3] << " ms";
  if (hz < 47.5 || period95 > 30)
    RCLCPP_WARN(get_logger(), "%s", text.str().c_str());
  else
    RCLCPP_INFO(get_logger(), "%s", text.str().c_str());
  timings_[0] = row;
  timing_size_ = 1;
  report_at_ = at;
}
}  // namespace noetix::policy
