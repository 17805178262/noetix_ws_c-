// Independent native C++ policy algorithms. ROS and ONNX bindings are kept outside the algorithms.
#pragma once
#include "noetix_policy_cpp/reference.hpp"
#include "noetix_policy_cpp/specs.hpp"

#include <memory>

namespace noetix::policy {

class Motion {
 public:
  Motion(const std::string& path, const std::vector<std::string>& joints);
  void reset() { heading_.reset(); }
  void align(const F4& quaternion);
  Pose frame(int index) const;
  size_t size() const { return poses_.size(); }

 private:
  std::vector<Pose> poses_;
  std::optional<F4> heading_;
};

// Policy contract: no ROS messages, no ONNX objects, no per-tick dynamic sizing.
// The inference adapter binds inputs exactly once after factory construction.
class IPolicy {
 public:
  explicit IPolicy(Spec config);
  virtual ~IPolicy() = default;
  IPolicy(const IPolicy&) = delete;
  IPolicy& operator=(const IPolicy&) = delete;

  const Spec config;
  std::vector<std::vector<float>> inputs;
  Joints last_action{}, target{};

  virtual void reset() = 0;
  virtual void reset_history() = 0;
  virtual void build(const State& state, const F3& command, double now) = 0;
  virtual void decode(const float* action, size_t count) = 0;

  // Only GMT supplies external reference. Others have no reference buffer.
  virtual Reference* reference() noexcept { return nullptr; }
  virtual const Reference* reference() const noexcept { return nullptr; }

 protected:
  void validate_state(const State& state) const;
  void decode_scaled_action(const float* action, size_t count, bool clip_walk);
};

class WalkPolicy final : public IPolicy {
 public:
  explicit WalkPolicy(Spec config);
  void reset() override;
  void reset_history() override;
  void build(const State& state, const F3& command, double now) override;
  void decode(const float* action, size_t count) override;

 private:
  std::array<std::array<float, 96>, 5> history_{};
  size_t head_ = 0;
  bool ready_ = false;
};

// Common *mathematics* for motion-tracking policies; no Kind branching in hot path.
class TrackingPolicy : public IPolicy {
 public:
  explicit TrackingPolicy(Spec config);
  void reset_history() override;
  void decode(const float* action, size_t count) override;

 protected:
  void build_tracking(const State& state, const std::array<Pose, 8>& frames,
                      const Pose& center, const F4& heading);
  bool history_ready() const noexcept { return ready_; }

 private:
  std::array<State, 17> history_{};
  std::array<Joints, 3> action_history_{};
  size_t history_head_ = 0;
  size_t action_head_ = 0;
  bool ready_ = false;
};

class GmtPolicy final : public TrackingPolicy {
 public:
  explicit GmtPolicy(Spec config);
  void reset() override;
  void build(const State& state, const F3& command, double now) override;
  Reference* reference() noexcept override { return &reference_; }
  const Reference* reference() const noexcept override { return &reference_; }

 private:
  Reference reference_;
};

class MotionPolicy final : public TrackingPolicy {
 public:
  MotionPolicy(Spec config, const std::string& motion_path);
  void reset() override;
  void build(const State& state, const F3& command, double now) override;
  void decode(const float* action, size_t count) override;

 private:
  Motion motion_;
  int motion_index_ = 0;
};

// 只在创建策略时查注册表，推理循环直接调用策略对象。
Spec policy_spec(const std::string& name);
std::unique_ptr<IPolicy> make_policy(Spec config, const std::string& motion_path = "");
V3 estimate_root_position(const State& state);

}  // namespace noetix::policy
