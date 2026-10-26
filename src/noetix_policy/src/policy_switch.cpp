// 中文说明：统一手柄切换器，保持Python版controls/policies配置、组合键优先和上升沿触发。
// 切换器不订阅LowState或GMT、不组织策略观测；各PolicyNode独立接收自己的输入。
// 模型在就绪前预加载；耗时构造在切换锁外，使用generation取消失能期间尚未完成的加载。
// GMT仅在首条真实命令准备好后接管；退出或lie/stand时释放控制，不在重新使能时自动恢复策略。
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/create_timer.hpp>
#include <set>

#include "noetix_policy_cpp/ros_policy.hpp"
namespace noetix::policy {
PolicySwitch::PolicySwitch(std::shared_ptr<rclcpp::Executor> executor,
                           const rclcpp::NodeOptions& options)
    : Node("noetix_policy_node", options), executor_(std::move(executor)) {
  auto share = ament_index_cpp::get_package_share_directory("noetix_policy");
  config_ = declare_parameter<std::string>("joystick_config", share + "/config/joystick.yaml");
  auto config = YAML::LoadFile(config_)["joystick"];
  require(!config["bindings"], "Use controls and policies sections");
  joy_topic_ = config["topic"].as<std::string>();
  std::set<std::vector<int>> seen;
  for (const std::string section : {"controls", "policies"})
    for (auto item : config[section]) {
      bool control = section == "controls";
      std::string field = control ? "action" : "policy", other = control ? "policy" : "action";
      require(bool(item[field]) && !item[other], "Invalid joystick binding target");
      Binding binding;
      binding.control = control;
      binding.target = item[field].as<std::string>();
      if (control)
        require(binding.target == "enable" || binding.target == "disable" ||
                    binding.target == "lie" || binding.target == "stand",
                "Unsupported control action");
      require(!(item["button"] && item["buttons"]), "Use either button or buttons");
      if (item["buttons"])
        binding.buttons = item["buttons"].as<std::vector<int>>();
      else
        binding.buttons = {item["button"].as<int>()};
      auto sorted = binding.buttons;
      std::sort(sorted.begin(), sorted.end());
      require(!sorted.empty() && sorted.front() >= 0 &&
                  std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end(),
              "Invalid joystick button list");
      require(seen.insert(sorted).second, "Duplicate joystick button binding");
      bindings_.push_back(binding);
    }
  std::stable_sort(bindings_.begin(), bindings_.end(), [](const Binding& a, const Binding& b) {
    return a.buttons.size() > b.buttons.size();
  });
  std::set<std::string> preloaded;
  for (const auto& binding : bindings_)
    if (!binding.control && preloaded.insert(binding.target).second) {
      try {
        auto value = spec(parse_kind(binding.target));
        load_model(value, share + "/models/" + value.model);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Policy preload failed: %s: %s", binding.target.c_str(),
                    e.what());
      }
    }
  control_pub_ = create_publisher<std_msgs::msg::String>("/noetix/control_request", 20);
}
void PolicySwitch::start() {
  auto weak =
      std::weak_ptr<PolicySwitch>(std::static_pointer_cast<PolicySwitch>(shared_from_this()));
  joy_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = joy_group_;
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      [weak](sensor_msgs::msg::Joy::ConstSharedPtr value) {
        if (auto self = weak.lock())
          self->joy(*value);
      },
      options);
  handoff_timer_ =
      rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(.005), [weak] {
        if (auto self = weak.lock())
          self->finish_handoff();
      });
  RCLCPP_INFO(get_logger(), "C++ joystick mode and policy switching ready");
}
void PolicySwitch::joy(const sensor_msgs::msg::Joy& value) {
  std::optional<Binding> selected;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (closing_)
      return;
    std::vector<const Binding*> held;
    std::set<int> reserved;
    for (const auto& binding : bindings_) {
      bool match = true;
      for (int button : binding.buttons)
        if (size_t(button) >= value.buttons.size() || !value.buttons[button])
          match = false;
      if (match) {
        held.push_back(&binding);
        if (binding.buttons.size() > 1)
          reserved.insert(binding.buttons.begin(), binding.buttons.end());
      }
    }
    for (auto binding : held) {
      if (binding->buttons.size() == 1 && reserved.count(binding->buttons[0]))
        continue;
      bool edge = false;
      for (int button : binding->buttons)
        if (size_t(button) >= previous_.size() || !previous_[button])
          edge = true;
      if (edge) {
        selected = *binding;
        break;
      }
    }
    previous_ = value.buttons;
  }
  if (selected) {
    if (selected->control)
      control(selected->target);
    else
      select(selected->target);
  }
}
void PolicySwitch::dispose(std::shared_ptr<PolicyNode>& node, bool release) {
  if (node) {
    node->shutdown(release);
    // Humble/FastDDS可能仍在另一线程的rmw_wait中持有节点实体。
    // 先停止发布并保留生命周期，主循环在执行器静止后移除和销毁。
    retired_.push_back(node);
    retired_ready_ = true;
    node.reset();
  }
}
void PolicySwitch::collect_retired() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  retired_.clear();
  retired_ready_ = false;
}
void PolicySwitch::detach_executor() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!executor_)
    return;
  for (auto& node : retired_)
    executor_->remove_node(node);
  if (current_)
    executor_->remove_node(current_);
  if (pending_)
    executor_->remove_node(pending_);
  executor_->remove_node(shared_from_this());
  executor_.reset();
}
void PolicySwitch::attach_executor(std::shared_ptr<rclcpp::Executor> executor) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  executor_ = std::move(executor);
  executor_->add_node(shared_from_this());
  if (current_)
    executor_->add_node(current_);
  if (pending_)
    executor_->add_node(pending_);
}
void PolicySwitch::clear(bool release) {
  ++generation_;
  dispose(pending_, false);
  dispose(current_, release);
}
void PolicySwitch::control(const std::string& action) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (closing_)
    return;
  if (action != "enable")
    clear(true);
  std_msgs::msg::String msg;
  msg.data = action;
  control_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Control request: %s", action.c_str());
}
void PolicySwitch::select(const std::string& name) {
  uint64_t generation;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (closing_)
      return;
    generation = ++generation_;
    dispose(pending_, false);
  }
  std::shared_ptr<PolicyNode> node;
  Kind kind;
  try {
    kind = parse_kind(name);
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", get_parameter("use_sim_time").as_bool()),
         rclcpp::Parameter("joystick_config", config_)});
    node = std::make_shared<PolicyNode>(kind, options, kind == Kind::gmt);
  } catch (const std::exception& e) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!closing_ && generation == generation_)
      RCLCPP_ERROR(get_logger(), "Failed to start policy: %s: %s", name.c_str(), e.what());
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (closing_ || generation != generation_) {
    node->shutdown(false);
    return;
  }
  try {
    if (kind == Kind::gmt) {
      pending_ = node;
      executor_->add_node(node);
      node->start();
      RCLCPP_INFO(get_logger(), "Policy preparing first command: %s", name.c_str());
    } else {
      clear(false);
      current_ = node;
      executor_->add_node(node);
      node->start();
      RCLCPP_INFO(get_logger(), "Policy selected: %s", name.c_str());
    }
  } catch (const std::exception& e) {
    if (pending_ == node)
      dispose(pending_, false);
    else if (current_ == node)
      dispose(current_, false);
    RCLCPP_ERROR(get_logger(), "Failed to attach policy: %s", e.what());
  }
}
void PolicySwitch::finish_handoff() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (closing_ || !pending_)
    return;
  if (!pending_->running()) {
    dispose(pending_, false);
    return;
  }
  if (!pending_->ready())
    return;
  auto next = pending_;
  pending_.reset();
  ++generation_;
  auto previous = current_;
  current_.reset();
  if (previous)
    previous->stop("Policy handoff", false);
  if (next->commit_handoff()) {
    current_ = next;
    RCLCPP_INFO(get_logger(), "Policy handoff completed");
  } else
    dispose(next, false);
  dispose(previous, false);
}
void PolicySwitch::shutdown() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (closing_)
    return;
  closing_ = true;
  if (handoff_timer_)
    handoff_timer_->cancel();
  clear(true);
}
PolicySwitch::~PolicySwitch() { shutdown(); }
}  // namespace noetix::policy
