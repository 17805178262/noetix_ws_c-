// walk策略：关节顺序、动作参数与观测构建集中在本文件。
#include "noetix_policy_cpp/core.hpp"

namespace noetix::policy {

// 返回与训练一致的模型契约；资源路径由运行配置提供。
Spec walk_spec() {
  Spec s;
  s.kind = Kind::walk;
  s.name = "walk";
  s.robot = "n3";
  s.output = "actions";
  s.names = {
      "l_hip_pitch_joint",   "r_hip_pitch_joint",   "waist_yaw_joint",     "l_hip_roll_joint",
      "r_hip_roll_joint",    "head_pitch_joint",    "l_arm_pitch_joint",   "r_arm_pitch_joint",
      "l_hip_yaw_joint",     "r_hip_yaw_joint",     "head_yaw_joint",      "l_arm_roll_joint",
      "r_arm_roll_joint",    "l_knee_pitch_joint",  "r_knee_pitch_joint",  "l_arm_yaw_joint",
      "r_arm_yaw_joint",     "l_ankle_pitch_joint", "r_ankle_pitch_joint", "l_elbow_pitch_joint",
      "r_elbow_pitch_joint", "l_ankle_roll_joint",  "r_ankle_roll_joint",  "l_hand_yaw_joint",
      "r_hand_yaw_joint",    "l_hand_pitch_joint",  "r_hand_pitch_joint",  "l_hand_roll_joint",
      "r_hand_roll_joint"};
  s.default_q = {-0.14949999999999999f,
                 -0.14949999999999999f,
                 0.0f,
                 0.0f,
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
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f,
                 0.0f};
  s.kp = {200.0f, 200.0f, 200.0f, 200.0f, 200.0f, 20.0f, 50.0f, 50.0f, 200.0f, 200.0f,
          20.0f,  50.0f,  50.0f,  200.0f, 200.0f, 50.0f, 50.0f, 50.0f, 50.0f,  50.0f,
          50.0f,  50.0f,  50.0f,  20.0f,  20.0f,  20.0f, 20.0f, 20.0f, 20.0f};
  s.kd = {15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 3.0f, 5.0f, 5.0f, 15.0f, 15.0f,
          3.0f,  5.0f,  5.0f,  15.0f, 15.0f, 5.0f, 5.0f, 4.0f, 4.0f,  5.0f,
          5.0f,  4.0f,  4.0f,  3.0f,  3.0f,  3.0f, 3.0f, 3.0f, 3.0f};
  s.action_scale = {0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,
                    0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f,
                    0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f};
  s.inputs = {{"obs", {1, 480}}};
  return s;
}

// 构造并校验本策略的输入输出契约。
WalkPolicy::WalkPolicy(Spec config) : IPolicy(std::move(config)) {
  require(this->config.kind == Kind::walk && this->config.dof() == 29 &&
              inputs.size() == 1 && inputs[0].size() == 480,
          "Invalid walk policy contract");
  reset();
}
// 清空本体及动作历史，为下一次执行重新初始化。
void WalkPolicy::reset_history() {
  ready_ = false;
  head_ = 0;
  last_action.fill(0);
}
// 重置本策略的历史和参考播放状态。
void WalkPolicy::reset() { reset_history(); }
// 根据当前反馈构建训练要求的观测，周期内不读配置或资源文件。
void WalkPolicy::build(const State& state, const F3& command, double /*now*/) {
  validate_state(state);
  require(command.allFinite(), "Joystick contains NaN/Inf");
  std::array<float, 96> frame{};
  size_t k = 0;
  for (float x : command)
    frame[k++] = x;
  for (float x : state.gyro)
    frame[k++] = x;
  for (float x : state.gravity)
    frame[k++] = x;
  for (int j = 0; j < config.dof(); ++j)
    frame[k++] = state.q[j] - config.default_q[j];
  for (int j = 0; j < config.dof(); ++j)
    frame[k++] = state.dq[j];
  for (int j = 0; j < config.dof(); ++j)
    frame[k++] = last_action[j];
  require(k == frame.size(), "Internal walk frame shape mismatch");
  if (!ready_) {
    history_.fill(frame);
    head_ = 0;
    ready_ = true;
  } else {
    head_ = (head_ + 1) % history_.size();
    history_[head_] = frame;
  }
  auto out = inputs[0].begin();
  for (size_t i = 1; i <= history_.size(); ++i)
    out = std::copy(history_[(head_ + i) % history_.size()].begin(),
                    history_[(head_ + i) % history_.size()].end(), out);
}
// 将网络动作还原为关节目标，并更新动作历史。
void WalkPolicy::decode(const float* action, size_t count) {
  decode_scaled_action(action, count, true);
}

}  // namespace noetix::policy
