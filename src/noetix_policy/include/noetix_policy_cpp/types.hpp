// 中文说明：C++策略的无ROS数据类型和四元数数学函数。
// 控制观测保持float32，参考时间轴与插值使用double；四元数始终为wxyz。
// 固定29关节容量、17帧本体历史和13帧参考窗口，使周期内不需要创建Python对象或扩容。
// 本文件不订阅话题、不连接驱动；所有非有限值和零四元数在进入控制计算前拒绝。
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
namespace noetix::policy {
constexpr int max_dof = 29;
using V3 = Eigen::Vector3d;
using Q4 = Eigen::Vector4d;
using F3 = Eigen::Vector3f;
using F4 = Eigen::Vector4f;
using Joints = std::array<float, max_dof>;
using DJoints = std::array<double, max_dof>;
enum class Kind { walk, gmt, motion };
struct InputSpec {
  std::string name;
  std::vector<int64_t> shape;
};
struct Spec {
  Kind kind;
  std::string name, robot, output;
  std::vector<std::string> names, transport;
  std::vector<float> default_q, kp, kd, action_scale;
  std::vector<InputSpec> inputs;
  int dof() const { return static_cast<int>(names.size()); }
};
struct State {
  Joints q{}, dq{};
  F4 quat{1, 0, 0, 0};
  F3 gyro = F3::Zero(), gravity{0, 0, -1};
  std::optional<V3> position;
};
struct Pose {
  V3 position = V3::Zero();
  Q4 quaternion{1, 0, 0, 0};
  DJoints joints{};
};
struct ReferenceNotReady : std::runtime_error {
  using std::runtime_error::runtime_error;
};
inline void require(bool ok, const std::string& error) {
  if (!ok)
    throw std::invalid_argument(error);
}
template <class T>
Eigen::Matrix<T, 4, 1> normalize(Eigen::Matrix<T, 4, 1> q) {
  T norm = q.norm();
  require(q.allFinite() && std::isfinite(norm) && norm >= T(1e-8), "Invalid quaternion");
  return q / norm;
}
template <class T>
Eigen::Matrix<T, 4, 1> conjugate(Eigen::Matrix<T, 4, 1> q) {
  q.template tail<3>() *= T(-1);
  return q;
}
template <class T>
Eigen::Matrix<T, 4, 1> multiply(const Eigen::Matrix<T, 4, 1>& a, const Eigen::Matrix<T, 4, 1>& b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
template <class T>
Eigen::Matrix<T, 3, 1> rotate(const Eigen::Matrix<T, 4, 1>& q, const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 1> u = q.template tail<3>(), t = T(2) * u.cross(v);
  return v + q[0] * t + u.cross(t);
}
inline Q4 yaw_quat(const Q4& input) {
  auto q = normalize(input);
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double hx = 1 - 2 * (y * y + z * z), hy = 2 * (x * y + w * z);
  if (std::hypot(hx, hy) <= .1) {
    hx = 2 * (x * z + w * y);
    hy = 2 * (y * z - w * x);
    if (2 * (x * z - w * y) >= 0) {
      hx = -hx;
      hy = -hy;
    }
  }
  double half = std::atan2(hy, hx) * .5;
  return {std::cos(half), 0, 0, std::sin(half)};
}
inline Q4 slerp(Q4 a, Q4 b, double t) {
  double dot = a.dot(b);
  if (dot < 0) {
    b = -b;
    dot = -dot;
  }
  if (dot > 1 - 1e-10)
    return normalize(Q4((1 - t) * a + t * b));
  double angle = std::acos(std::clamp(dot, -1., 1.));
  return normalize(Q4((std::sin((1 - t) * angle) * a + std::sin(t * angle) * b) / std::sin(angle)));
}
inline V3 angular_velocity(const Q4& a, const Q4& b) {
  Q4 d = normalize(multiply(b, conjugate(a)));
  if (d[0] < 0)
    d = -d;
  double sine = d.tail<3>().norm();
  if (sine < 1e-12)
    return V3::Zero();
  return d.tail<3>() * (2 * std::atan2(sine, d[0]) / sine);
}
inline Pose mix(const Pose& a, const Pose& b, double t, int dof) {
  Pose p;
  p.position = (1 - t) * a.position + t * b.position;
  p.quaternion = slerp(a.quaternion, b.quaternion, t);
  for (int j = 0; j < dof; ++j)
    p.joints[j] = (1 - t) * a.joints[j] + t * b.joints[j];
  return p;
}
}  // namespace noetix::policy
