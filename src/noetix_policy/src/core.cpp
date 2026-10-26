// Common policy math and history processing. The concrete strategies own their state.
// Floating point operations and observation order follow the original training contract.
#include "noetix_policy_cpp/core.hpp"

namespace noetix::policy {
namespace {
constexpr std::array<int, 7> lags{0, 1, 2, 3, 4, 8, 16};
constexpr std::array<int, 8> future_offsets{-8, -4, -2, 0, 1, 2, 3, 4};

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

F4 gmt_heading(const Pose& center) {
  // Keep float32 -> float64 cast order for parity with the Python GMT observations.
  return normalize(F4(yaw_quat(Q4(center.quaternion.cast<float>().cast<double>())).cast<float>()));
}

F4 motion_heading(const Pose& center) {
  F4 q = normalize(F4(center.quaternion.cast<float>()));
  F3 x = rotate(q, F3(1, 0, 0)), z = rotate(q, F3(0, 0, 1));
  F3 h = x.head<2>().norm() > .1f ? x : (x[2] < 0 ? z : F3(-z));
  double half = double(std::atan2(h[1], h[0])) * .5;
  return normalize(F4(float(std::cos(half)), 0, 0, float(std::sin(half))));
}
}  // namespace

IPolicy::IPolicy(Spec value) : config(std::move(value)) {
  inputs.reserve(config.inputs.size());
  for (const auto& input : config.inputs) {
    size_t size = 1;
    for (auto dim : input.shape) {
      require(dim > 0, "Invalid ONNX input shape");
      size *= size_t(dim);
    }
    inputs.emplace_back(size, 0.f);
  }
}

void IPolicy::validate_state(const State& state) const {
  require(state.quat.allFinite() && state.gyro.allFinite() && state.gravity.allFinite(),
          "Invalid state");
  for (int j = 0; j < config.dof(); ++j)
    require(std::isfinite(state.q[j]) && std::isfinite(state.dq[j]), "Invalid joint state");
}

void IPolicy::decode_scaled_action(const float* action, size_t count, bool clip_walk) {
  require(action != nullptr && count == size_t(config.dof()), "Wrong action count");
  // Validate the entire action before changing any controller state.
  for (size_t j = 0; j < count; ++j)
    require(std::isfinite(action[j]), "Action contains NaN/Inf");
  for (size_t j = 0; j < count; ++j) {
    float a = clip_walk ? std::clamp(action[j], -18.f, 18.f) : action[j];
    require(std::isfinite(config.default_q[j] + a * config.action_scale[j]),
            "Target contains NaN/Inf");
  }
  for (size_t j = 0; j < count; ++j) {
    last_action[j] = clip_walk ? std::clamp(action[j], -18.f, 18.f) : action[j];
    target[j] = config.default_q[j] + last_action[j] * config.action_scale[j];
  }
}

WalkPolicy::WalkPolicy(Spec config) : IPolicy(std::move(config)) {
  require(this->config.kind == Kind::walk && this->config.dof() == 29 &&
              inputs.size() == 1 && inputs[0].size() == 480,
          "Invalid walk policy contract");
  reset();
}
void WalkPolicy::reset_history() {
  ready_ = false;
  head_ = 0;
  last_action.fill(0);
}
void WalkPolicy::reset() { reset_history(); }
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
void WalkPolicy::decode(const float* action, size_t count) {
  decode_scaled_action(action, count, true);
}

TrackingPolicy::TrackingPolicy(Spec config) : IPolicy(std::move(config)) {
  require((this->config.kind == Kind::gmt || this->config.kind == Kind::motion) &&
              inputs.size() == 2,
          "Invalid tracking policy contract");
  reset_history();
}
void TrackingPolicy::reset_history() {
  ready_ = false;
  history_head_ = 0;
  action_head_ = 0;
  last_action.fill(0);
  for (auto& history : action_history_)
    history.fill(0);
}
void TrackingPolicy::decode(const float* action, size_t count) {
  decode_scaled_action(action, count, false);
  action_head_ = (action_head_ + 1) % action_history_.size();
  action_history_[action_head_] = last_action;
}
void TrackingPolicy::build_tracking(const State& state, const std::array<Pose, 8>& frames,
                                    const Pose& center, const F4& heading) {
  if (!ready_) {
    history_.fill(state);
    history_head_ = 0;
    ready_ = true;
  } else {
    history_head_ = (history_head_ + 1) % history_.size();
    history_[history_head_] = state;
  }
  size_t k = 0;
  auto& policy = inputs[1];
  for (int term = 0; term < 4; ++term)
    for (int lag : lags) {
      const auto& h = history_[(history_head_ + history_.size() - size_t(lag)) % history_.size()];
      if (term < 2) {
        const auto& v = term == 0 ? h.gyro : h.gravity;
        for (float x : v)
          policy[k++] = x;
      } else {
        const auto& v = term == 2 ? h.q : h.dq;
        for (int j = 0; j < config.dof(); ++j)
          policy[k++] = v[j];
      }
    }
  for (int lag = 0; lag < 3; ++lag) {
    const auto& a = action_history_[(action_head_ + action_history_.size() - size_t(lag)) %
                                    action_history_.size()];
    for (int j = 0; j < config.dof(); ++j)
      policy[k++] = a[j];
  }
  require(k == policy.size(), "Internal policy shape mismatch");
  F3 anchor = center.position.cast<float>();
  anchor[2] = 0;
  auto& output = inputs[0];
  k = 0;
  for (const auto& frame : frames) {
    F3 local = rotate(conjugate(heading), F3(frame.position.cast<float>() - anchor));
    for (float x : local)
      output[k++] = x;
  }
  for (const auto& frame : frames) {
    F4 q = normalize(multiply(conjugate(state.quat), F4(frame.quaternion.cast<float>())));
    float w = q[0], x = q[1], y = q[2], z = q[3];
    for (float v : {1.f - 2.f * (y * y + z * z), 2.f * (x * y - z * w),
                    2.f * (x * z + y * w), 2.f * (x * y + z * w),
                    1.f - 2.f * (x * x + z * z), 2.f * (y * z - x * w)})
      output[k++] = v;
  }
  for (const auto& frame : frames)
    for (int j = 0; j < config.dof(); ++j)
      output[k++] = float(frame.joints[j]);
  require(k == output.size(), "Internal command shape mismatch");
}

GmtPolicy::GmtPolicy(Spec config)
    : TrackingPolicy(std::move(config)), reference_(transport_indices(this->config)) {
  require(this->config.kind == Kind::gmt && inputs[0].size() == 304 &&
              inputs[1].size() == 535,
          "Invalid GMT policy contract");
}
void GmtPolicy::reset() {
  reset_history();
  reference_ = Reference(transport_indices(config));
}
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

MotionPolicy::MotionPolicy(Spec config, const std::string& path)
    : TrackingPolicy(std::move(config)), motion_(path, this->config.names) {
  require(this->config.kind == Kind::motion && inputs[0].size() == 240 &&
              inputs[1].size() == 399,
          "Invalid motion policy contract");
  reset();
}
void MotionPolicy::reset() {
  reset_history();
  motion_index_ = 0;
  motion_.reset();
}
void MotionPolicy::build(const State& state, const F3& /*command*/, double /*now*/) {
  validate_state(state);
  motion_.align(state.quat);
  std::array<Pose, 8> frames;
  for (size_t i = 0; i < future_offsets.size(); ++i)
    frames[i] = motion_.frame(motion_index_ + future_offsets[i]);
  auto center = motion_.frame(motion_index_);
  build_tracking(state, frames, center, motion_heading(center));
}
void MotionPolicy::decode(const float* action, size_t count) {
  TrackingPolicy::decode(action, count);
  ++motion_index_;
}

std::unique_ptr<IPolicy> make_policy(Spec config, const std::string& motion_path) {
  switch (config.kind) {
    case Kind::walk: return std::make_unique<WalkPolicy>(std::move(config));
    case Kind::gmt: return std::make_unique<GmtPolicy>(std::move(config));
    case Kind::motion: return std::make_unique<MotionPolicy>(std::move(config), motion_path);
  }
  throw std::invalid_argument("Unsupported policy kind");
}

V3 estimate_root_position(const State& state) {
  if (state.position)
    return *state.position;
  V3 up = -state.gravity.cast<double>();
  up.normalize();
  double height = -1e9;
  const std::array<V3, 6> axes{V3(0, 1, 0), V3(1, 0, 0), V3(0, 0, 1),
                               V3(0, 1, 0), V3(0, 1, 0), V3(1, 0, 0)};
  for (int side : {1, -1}) {
    int start = side == 1 ? 0 : 6;
    std::array<V3, 6> offsets{V3(0, side * .071, .05215), V3(0, side * .054, -.0899),
                              V3(0, 0, -.1269), V3(0, 0, -.1631), V3(0, 0, -.34), V3::Zero()};
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    V3 position = V3::Zero();
    for (int j = 0; j < 6; ++j) {
      position += rotation * offsets[j];
      const auto& a = axes[j];
      Eigen::Matrix3d skew;
      skew << 0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0;
      float angle = state.q[start + j];
      rotation = rotation * (Eigen::Matrix3d::Identity() + double(std::sin(angle)) * skew +
                             (1 - double(std::cos(angle))) * (skew * skew));
    }
    V3 center = position + rotation * V3(side == 1 ? .04 : .0475, side * .0015, -.056);
    double extent =
        (up.transpose() * rotation).cwiseAbs().dot(V3(side == 1 ? .11 : .1105, .0495, .01));
    height = std::max(height, -up.dot(center) + extent);
  }
  return {0, 0, height};
}
}  // namespace noetix::policy
