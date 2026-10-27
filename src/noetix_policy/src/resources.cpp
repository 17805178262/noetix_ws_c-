// 从独立配置读取模型与轨迹路径，不从文件名推断训练契约。
#include "noetix_policy_cpp/resources.hpp"
#include "noetix_policy_cpp/types.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace noetix::policy {
namespace {
// 将package URI、绝对路径或配置旁的相对路径转换为实际文件。
std::string resolve(const YAML::Node& entry, const std::filesystem::path& directory) {
  require(entry && entry.IsScalar(), "Resource path must be a non-empty string");
  auto value = entry.as<std::string>();
  require(!value.empty(), "Resource path must not be empty");
  std::filesystem::path path;
  const std::string prefix = "package://";
  if (value.compare(0, prefix.size(), prefix) == 0) {
    auto slash = value.find('/', prefix.size());
    require(slash != std::string::npos && slash > prefix.size(), "Invalid package URI: " + value);
    path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
        value.substr(prefix.size(), slash - prefix.size()))) / value.substr(slash + 1);
  } else {
    path = value;
    if (path.is_relative())
      path = directory / path;
  }
  path = std::filesystem::absolute(path).lexically_normal();
  require(std::filesystem::is_regular_file(path), "Resource file not found: " + path.string());
  return path.string();
}
}  // namespace
// 每次进入策略重新读配置，已在运行的策略继续使用自己的资源快照。
PolicyResources load_resources(const std::string& config, const std::string& name,
                               bool needs_motion) {
  auto root = YAML::LoadFile(config);
  YAML::Node entry(YAML::NodeType::Undefined);
  for (auto item : root["joystick"]["policies"]) {
    auto target = item["policy"].as<std::string>();
    if (target == name || target == "noetix_policy.policies." + name) {
      require(!entry, "Duplicate policy resource definition: " + name);
      entry.reset(item);
    }
  }
  require(entry && entry.IsMap(), "Missing policy resources: " + name + " in " + config);
  auto directory = std::filesystem::absolute(config).parent_path();
  PolicyResources resources;
  resources.model = resolve(entry["model"], directory);
  if (needs_motion)
    resources.motion = resolve(entry["motion"], directory);
  return resources;
}
}  // namespace noetix::policy
