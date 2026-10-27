// motion策略：关节顺序、动作参数与观测构建集中在本文件。
#include "noetix_policy_cpp/core.hpp"

namespace noetix::policy {
namespace {
constexpr std::array<int, 8> future_offsets{-8, -4, -2, 0, 1, 2, 3, 4};
// 提取离线轨迹朝向，保持原训练部署定义。
F4 motion_heading(const Pose& center) {
  F4 q = normalize(F4(center.quaternion.cast<float>()));
  F3 x = rotate(q, F3(1, 0, 0)), z = rotate(q, F3(0, 0, 1));
  F3 h = x.head<2>().norm() > .1f ? x : (x[2] < 0 ? z : F3(-z));
  double half = double(std::atan2(h[1], h[0])) * .5;
  return normalize(F4(float(std::cos(half)), 0, 0, float(std::sin(half))));
}
}  // namespace

// 返回与训练一致的模型契约；资源路径由运行配置提供。
Spec motion_spec() {
  Spec s;
  s.kind = Kind::motion;
  s.name = "motion";
  s.robot = "bumi4340";
  s.output = "action";
  s.names = {"l_leg_pitch_joint",   "r_leg_pitch_joint",   "waist_yaw_joint",
             "l_leg_roll_joint",    "r_leg_roll_joint",    "l_arm_pitch_joint",
             "r_arm_pitch_joint",   "l_leg_yaw_joint",     "r_leg_yaw_joint",
             "l_arm_roll_joint",    "r_arm_roll_joint",    "l_knee_pitch_joint",
             "r_knee_pitch_joint",  "l_arm_yaw_joint",     "r_arm_yaw_joint",
             "l_ankle_pitch_joint", "r_ankle_pitch_joint", "l_elbow_pitch_joint",
             "r_elbow_pitch_joint", "l_ankle_roll_joint",  "r_ankle_roll_joint"};
  s.default_q = {-0.14949999999999999f,
                 -0.14949999999999999f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.29999999999999999f,
                 -0.29999999999999999f,
                 0.32150000000000001f,
                 0.32150000000000001f,
                 0.0f,
                 0.0f,
                 -0.17199999999999999f,
                 -0.17199999999999999f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f};
  s.kp = {45.0f, 45.0f, 53.0f, 45.0f, 45.0f,  8.0f,   8.0f, 35.0f, 35.0f,  8.0f,  8.0f,
          45.0f, 45.0f, 8.0f,  8.0f,  15.75f, 15.75f, 8.0f, 8.0f,  15.75f, 15.75f};
  s.kd = {3.0f,
          3.0f,
          3.3999999999999999f,
          3.0f,
          3.0f,
          0.40000000000000002f,
          0.40000000000000002f,
          2.5f,
          2.5f,
          0.40000000000000002f,
          0.40000000000000002f,
          2.0f,
          2.0f,
          0.40000000000000002f,
          0.40000000000000002f,
          1.125f,
          1.125f,
          0.40000000000000002f,
          0.40000000000000002f,
          1.125f,
          1.125f};
  s.action_scale = {0.44999998807907104f,
                    0.44999998807907104f,
                    0.44999998807907104f,
                    0.34999999403953552f,
                    0.34999999403953552f,
                    0.40000000596046448f,
                    0.40000000596046448f,
                    0.44999998807907104f,
                    0.44999998807907104f,
                    0.40000000596046448f,
                    0.40000000596046448f,
                    0.34999999403953552f,
                    0.34999999403953552f,
                    0.40000000596046448f,
                    0.40000000596046448f,
                    0.30000001192092896f,
                    0.30000001192092896f,
                    0.40000000596046448f,
                    0.40000000596046448f,
                    0.25f,
                    0.25f};
  s.inputs = {{"command", {240}}, {"policy", {399}}};
  return s;
}

// 构造并校验本策略的输入输出契约。
MotionPolicy::MotionPolicy(Spec config, const std::string& path)
    : TrackingPolicy(std::move(config)), motion_(path, this->config.names) {
  require(this->config.kind == Kind::motion && inputs[0].size() == 240 &&
              inputs[1].size() == 399,
          "Invalid motion policy contract");
  reset();
}
// 重置本策略的历史和参考播放状态。
void MotionPolicy::reset() {
  reset_history();
  motion_index_ = 0;
  motion_.reset();
}
// 根据当前反馈构建训练要求的观测，周期内不读配置或资源文件。
void MotionPolicy::build(const State& state, const F3& /*command*/, double /*now*/) {
  validate_state(state);
  motion_.align(state.quat);
  std::array<Pose, 8> frames;
  for (size_t i = 0; i < future_offsets.size(); ++i)
    frames[i] = motion_.frame(motion_index_ + future_offsets[i]);
  auto center = motion_.frame(motion_index_);
  build_tracking(state, frames, center, motion_heading(center));
}
// 将网络动作还原为关节目标，并更新动作历史。
void MotionPolicy::decode(const float* action, size_t count) {
  TrackingPolicy::decode(action, count);
  ++motion_index_;
}

}  // namespace noetix::policy
