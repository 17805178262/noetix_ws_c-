#pragma once

// Standalone subset of the original N3HW::MotorData. No ROS or Eigen needed.
// Kept field-for-field compatible with n3_hw/legged_n3_hw/include/Types.h.
struct MotorData {
  double pos_{0.0};
  double vel_{0.0};
  double tau_{0.0};
  double pos_des_{0.0};
  double vel_des_{0.0};
  double kp_{0.0};
  double kd_{0.0};
  double ff_{0.0};
};
