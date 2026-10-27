// gmt策略：关节顺序、动作参数与观测构建集中在本文件。
#include "noetix_policy_cpp/core.hpp"

namespace noetix::policy {
namespace {
constexpr std::array<int, 8> future_offsets{-8, -4, -2, 0, 1, 2, 3, 4};
// 将传输关节顺序映射为模型要求的顺序，只在初始化或重置时计算。
std::vector<int> transport_indices(const Spec& config) {
  std::vector<int> indices;
  indices.reserve(config.names.size());
  for (const auto& name : config.names) {
    auto it = std::find(config.transport.begin(), config.transport.end(), name);
    require(it != config.transport.end(), "Missing transport joint");
    indices.push_back(int(it - config.transport.begin()));
  }
  return indices;
}

// 提取GMT参考朝向，保持训练使用的浮点转换顺序。
F4 gmt_heading(const Pose& center) {
  // 先转float32再转float64，与已有观测的数值顺序一致。
  return normalize(F4(yaw_quat(Q4(center.quaternion.cast<float>().cast<double>())).cast<float>()));
}

}  // namespace

// 返回与训练一致的模型契约；资源路径由运行配置提供。
Spec gmt_spec() {
  Spec s;
  s.kind = Kind::gmt;
  s.name = "gmt";
  s.robot = "n3";
  s.output = "action";
  s.names = {
      "l_hip_pitch_joint",   "l_hip_roll_joint",   "l_hip_yaw_joint",     "l_knee_pitch_joint",
      "l_ankle_pitch_joint", "l_ankle_roll_joint", "r_hip_pitch_joint",   "r_hip_roll_joint",
      "r_hip_yaw_joint",     "r_knee_pitch_joint", "r_ankle_pitch_joint", "r_ankle_roll_joint",
      "waist_yaw_joint",     "l_arm_pitch_joint",  "l_arm_roll_joint",    "l_arm_yaw_joint",
      "l_elbow_pitch_joint", "l_hand_yaw_joint",   "l_hand_pitch_joint",  "l_hand_roll_joint",
      "r_arm_pitch_joint",   "r_arm_roll_joint",   "r_arm_yaw_joint",     "r_elbow_pitch_joint",
      "r_hand_yaw_joint",    "r_hand_pitch_joint", "r_hand_roll_joint",   "head_pitch_joint",
      "head_yaw_joint"};
  s.default_q = {-0.14949999999999999f,
                 0.0f,
                 0.0f,
                 0.32150000000000001f,
                 -0.17199999999999999f,
                 0.0f,
                 -0.14949999999999999f,
                 0.0f,
                 0.0f,
                 0.32150000000000001f,
                 -0.17199999999999999f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.20000000000000001f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 -0.20000000000000001f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f};
  s.kp = {200.0f, 200.0f, 200.0f, 200.0f, 80.0f, 50.0f, 200.0f, 200.0f, 200.0f, 200.0f,
          80.0f,  50.0f,  80.0f,  50.0f,  50.0f, 50.0f, 50.0f,  20.0f,  20.0f,  20.0f,
          50.0f,  50.0f,  50.0f,  50.0f,  20.0f, 20.0f, 20.0f,  20.0f,  20.0f};
  s.kd = {15.0f, 15.0f, 15.0f, 15.0f, 4.0f, 2.0f, 15.0f, 15.0f, 15.0f, 15.0f,
          4.0f,  2.0f,  3.0f,  3.0f,  3.0f, 3.0f, 3.0f,  1.0f,  1.0f,  1.0f,
          3.0f,  3.0f,  3.0f,  3.0f,  1.0f, 1.0f, 1.0f,  3.0f,  3.0f};
  s.action_scale = {0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,
                    0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,
                    0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f};
  s.inputs = {{"command", {304}}, {"policy", {535}}};
  s.transport = {
      "l_hip_pitch_joint",   "r_hip_pitch_joint",   "waist_yaw_joint",     "l_hip_roll_joint",
      "r_hip_roll_joint",    "head_pitch_joint",    "l_arm_pitch_joint",   "r_arm_pitch_joint",
      "l_hip_yaw_joint",     "r_hip_yaw_joint",     "head_yaw_joint",      "l_arm_roll_joint",
      "r_arm_roll_joint",    "l_knee_pitch_joint",  "r_knee_pitch_joint",  "l_arm_yaw_joint",
      "r_arm_yaw_joint",     "l_ankle_pitch_joint", "r_ankle_pitch_joint", "l_elbow_pitch_joint",
      "r_elbow_pitch_joint", "l_ankle_roll_joint",  "r_ankle_roll_joint",  "l_hand_yaw_joint",
      "r_hand_yaw_joint",    "l_hand_pitch_joint",  "r_hand_pitch_joint",  "l_hand_roll_joint",
      "r_hand_roll_joint"};
  return s;
}

// 构造并校验本策略的输入输出契约。
GmtPolicy::GmtPolicy(Spec config)
    : TrackingPolicy(std::move(config)), reference_(transport_indices(this->config)) {
  require(this->config.kind == Kind::gmt && inputs[0].size() == 304 &&
              inputs[1].size() == 535,
          "Invalid GMT policy contract");
}
// 重置本策略的历史和参考播放状态。
void GmtPolicy::reset() {
  reset_history();
  reference_ = Reference(transport_indices(config));
}
// 根据当前反馈构建训练要求的观测，周期内不读配置或资源文件。
void GmtPolicy::build(const State& state, const F3& /*command*/, double now) {
  validate_state(state);
  reference_.align(state.quat.cast<double>(), state.position);
  if (!history_ready()) {
    Pose start;
    start.position = estimate_root_position(state);
    start.quaternion = state.quat.cast<double>();
    for (int j = 0; j < config.dof(); ++j)
      start.joints[j] = state.q[j];
    reference_.seed(start);
  }
  reference_.sample(now);
  std::array<Pose, 8> frames;
  for (size_t i = 0; i < future_offsets.size(); ++i)
    frames[i] = reference_.frame(future_offsets[i]);
  auto center = reference_.frame();
  build_tracking(state, frames, center, gmt_heading(center));
}

}  // namespace noetix::policy
