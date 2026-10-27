// 运行资源解析只在策略创建时执行，不进入控制循环。
#pragma once
#include <string>
namespace noetix::policy {
struct PolicyResources {
  std::string model, motion;
};
PolicyResources load_resources(const std::string& config, const std::string& name,
                               bool needs_motion);
}  // namespace noetix::policy
