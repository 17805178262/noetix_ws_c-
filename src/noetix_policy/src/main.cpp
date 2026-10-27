// 中文说明：C++统一启动入口，编译为policy_node和walk/gmt/motion三个独立可执行文件。
// ROS2执行器使用四个工作线程；GMT另有周期线程。信号只置退出标志，先停止策略并等待推理结束，
// 再销毁发布器和关闭ROS上下文，保证正常退出时仍能发布一次零增益释放命令。
#include <csignal>
#include <iostream>

#include "noetix_policy_cpp/ros_policy.hpp"
namespace {
volatile std::sig_atomic_t quitting = 0;
void on_signal(int) { quitting = 1; }
}  // namespace
int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  auto executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
  std::shared_ptr<noetix::policy::PolicySwitch> manager;
  std::shared_ptr<noetix::policy::PolicyNode> policy;
  std::thread spinner;
  std::atomic<bool> spin_failed{false};
  int result = 0;
  try {
#ifdef NOETIX_STANDALONE_KIND
    policy = std::make_shared<noetix::policy::PolicyNode>(
        NOETIX_STANDALONE_KIND, rclcpp::NodeOptions());
    executor->add_node(policy);
    policy->start();
#else
    manager = std::make_shared<noetix::policy::PolicySwitch>(executor);
    executor->add_node(manager);
    manager->start();
#endif
    auto spin = [&] {
      try {
        executor->spin();
      } catch (const std::exception& e) {
        std::cerr << "C++ executor error: " << e.what() << '\n';
        spin_failed = true;
      }
    };
    auto start_spin = [&] {
      spinner = std::thread(spin);
      // 避免SIGINT或再次回收发生在spin真正启动之前，使cancel被随后启动覆盖。
      while (!executor->is_spinning() && !spin_failed)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };
    start_spin();
    while (!quitting && rclcpp::ok() && !spin_failed) {
      if (manager && manager->has_retired()) {
        // 暂停并等待执行器工作线程结束后，才释放旧节点的DDS实体。
        // GMT的独立控制线程持续运行；这不是等待推理或停止新策略。
        executor->cancel();
        spinner.join();
        manager->detach_executor();
        // Humble还会缓存旧GuardCondition；整体销毁等待集后才释放旧节点。
        executor.reset();
        manager->collect_retired();
        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 4);
        manager->attach_executor(executor);
        if (!quitting && rclcpp::ok() && !spin_failed)
          start_spin();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (spin_failed)
      result = 1;
  } catch (const std::exception& e) {
    std::cerr << "C++ policy failed: " << e.what() << '\n';
    result = 1;
  }
  executor->cancel();
  if (spinner.joinable())
    spinner.join();
  if (manager) {
    manager->shutdown();
    manager->detach_executor();
    executor.reset();
    manager->collect_retired();
  }
  if (policy)
    policy->shutdown(true);
  executor.reset();
  manager.reset();
  policy.reset();
  rclcpp::shutdown();
  return result;
}
