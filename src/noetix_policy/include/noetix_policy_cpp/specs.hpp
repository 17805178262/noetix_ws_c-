// 中文说明：本文件固定保存当前Python三个内置策略的模型契约与硬件参数。
// 关节顺序、默认角、PD、动作倍率和输入维度逐项来自同目录层级的Python基线，
// C++运行时不导入Python；这些常量用于观测、动作历史及电机坐标映射，禁止按机器人名称猜测。
#pragma once
#include "noetix_policy_cpp/types.hpp"
namespace noetix::policy {
inline Spec walk_spec() {
  Spec s;
  s.kind = Kind::walk;
  s.name = "walk";
  s.robot = "n3";
  s.model = "walk.onnx";
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
inline Spec gmt_spec() {
  Spec s;
  s.kind = Kind::gmt;
  s.name = "gmt";
  s.robot = "n3";
  s.model = "gmt.onnx";
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
inline Spec motion_spec() {
  Spec s;
  s.kind = Kind::motion;
  s.name = "motion";
  s.robot = "bumi4340";
  s.model = "motion.onnx";
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
inline Spec spec(Kind k) {
  return k == Kind::walk ? walk_spec() : k == Kind::gmt ? gmt_spec() : motion_spec();
}
}  // namespace noetix::policy
