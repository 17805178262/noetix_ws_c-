// 策略注册表：新增部署实现时只在这里增加名称、训练契约和创建函数。
// Kind仅表示walk/GMT/motion的数据接入方式，不再负责选择模型文件。
#include "noetix_policy_cpp/core.hpp"

namespace noetix::policy {
namespace {
// 适配不需要motion文件的策略构造函数。
template <class Policy>
std::unique_ptr<IPolicy> create(Spec config, const std::string&) {
  return std::make_unique<Policy>(std::move(config));
}
// motion策略在构造时一次性读取轨迹，播放时只访问内存。
std::unique_ptr<IPolicy> create_motion(Spec config, const std::string& path) {
  return std::make_unique<MotionPolicy>(std::move(config), path);
}
struct Registration {
  const char* name;
  Spec (*spec)();
  std::unique_ptr<IPolicy> (*create)(Spec, const std::string&);
};
const Registration registrations[] = {
    {"walk", walk_spec, create<WalkPolicy>},
    {"gmt", gmt_spec, create<GmtPolicy>},
    {"motion", motion_spec, create_motion},
};
// 接受简短名称，也保留旧遥控器配置的名称写法。
const Registration& find(const std::string& value) {
  const std::string prefix = "noetix_policy.policies.";
  auto name = value.compare(0, prefix.size(), prefix) == 0 ? value.substr(prefix.size()) : value;
  for (const auto& item : registrations)
    if (name == item.name)
      return item;
  throw std::invalid_argument("Unknown C++ policy: " + value);
}
}  // namespace
// 创建训练参数副本，绑定注册名称，供资源配置与日志使用。
Spec policy_spec(const std::string& name) {
  const auto& item = find(name);
  auto value = item.spec();
  value.name = item.name;
  return value;
}
// 根据注册名称创建策略；查表只发生在初始化或切换时。
std::unique_ptr<IPolicy> make_policy(Spec config, const std::string& motion_path) {
  return find(config.name).create(std::move(config), motion_path);
}
}  // namespace noetix::policy
