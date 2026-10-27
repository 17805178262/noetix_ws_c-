// 三个示例的训练契约入口，定义和观测实现放在各自策略文件中。
#pragma once
#include "noetix_policy_cpp/types.hpp"
namespace noetix::policy {
Spec walk_spec();
Spec gmt_spec();
Spec motion_spec();
}  // namespace noetix::policy
