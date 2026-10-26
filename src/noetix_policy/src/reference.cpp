// 中文说明：实现GMT参考缓冲的确定性状态机；数学和阈值对应Python teleoperation_reference.py。
// 稳态以二分查找直接从固定环形缓冲插值13帧，不复制整个参考序列，不计算模型未使用的速度字段。
// 首次窗口不足抛ReferenceNotReady；断流后保持最后输出中心姿态；恢复保留本体和动作历史。
#include "noetix_policy_cpp/reference.hpp"

#include <set>
namespace noetix::policy {
Reference::Reference(std::vector<int> indices, ReferenceOptions options)
    : indices_(std::move(indices)), options_(options), blend_time_(options.blend) {
  require(!indices_.empty() && indices_.size() <= max_dof, "Invalid reference dof");
  std::set<int> unique(indices_.begin(), indices_.end());
  require(unique.size() == indices_.size() && *unique.begin() == 0 &&
              *unique.rbegin() == int(indices_.size()) - 1,
          "Invalid reference order");
  for (double v : {options.dt, options.stall, options.blend, options.startup, options.jump_distance,
                   options.jump_yaw, options.jump_joint})
    require(std::isfinite(v) && v > 0, "Invalid reference option");
  require(std::isfinite(options.jitter) && options.jitter >= 0 && std::isfinite(options.recovery) &&
              options.recovery >= 0,
          "Invalid recovery option");
}
bool Reference::reject(const std::string& reason) {
  ++rejected;
  last_rejection = reason;
  return false;
}
void Reference::seed(const Pose& pose) {
  if (last_)
    return;
  require(pose.position.allFinite(), "Invalid initial position");
  for (size_t j = 0; j < indices_.size(); ++j)
    require(std::isfinite(pose.joints[j]), "Invalid initial joints");
  initial_ = pose;
  initial_->quaternion = normalize(pose.quaternion);
  blend_from_ = initial_;
  blend_time_ = options_.startup;
  starting_ = true;
}
void Reference::align(const Q4& q, const std::optional<V3>& position) {
  robot_heading_ = yaw_quat(q);
  robot_position_.reset();
  if (position) {
    require(position->allFinite(), "Invalid robot position");
    if (!position_offset_) {
      position_offset_ = V3::Zero();
      if (last_)
        position_offset_->head<2>() = last_->position.head<2>() - position->head<2>();
    }
    robot_position_ = *position + *position_offset_;
  }
}
void Reference::new_segment(const std::string& reason) {
  head_ = count_ = offset_head_ = offset_count_ = 0;
  fix_.reset();
  cursor_.reset();
  translation_.setZero();
  blend_start_.reset();
  blend_from_ = last_ ? last_ : initial_;
  blend_time_ = starting_ ? options_.startup : options_.blend;
  recovery_since_.reset();
  rewind_.reset();
  ++segments;
  last_segment_reason = reason;
  status = last_ ? "refilling" : "waiting";
}
void Reference::append(const TimedPose& value) {
  if (status == "holding" || status == "refilling")
    if (!recovery_since_ || !last_received_ ||
        value.received - *last_received_ > std::max(.1, 4 * options_.dt))
      recovery_since_ = value.received;
  if (count_ == frames_.size()) {
    head_ = (head_ + 1) % frames_.size();
    --count_;
  }
  frames_[(head_ + count_++) % frames_.size()] = value;
  offsets_[offset_head_] = value.received - value.stamp;
  offset_head_ = (offset_head_ + 1) % offsets_.size();
  offset_count_ = std::min(offset_count_ + 1, offsets_.size());
  last_received_ = value.received;
}
bool Reference::push(const std::vector<double>& values, double received,
                     const std::string& source) {
  TimedPose value;
  try {
    require(values.size() == 14 + indices_.size() && std::isfinite(received),
            "Invalid GMT frame width/time");
    for (double v : values)
      require(std::isfinite(v), "GMT frame contains NaN/Inf");
    value.stamp = values[0];
    value.received = received;
    value.pose.position = V3(values[1], values[2], values[3]);
    value.pose.quaternion = normalize(Q4(values[4], values[5], values[6], values[7]));
    for (size_t j = 0; j < indices_.size(); ++j)
      value.pose.joints[j] = values[14 + indices_[j]];
  } catch (const std::exception& e) {
    return reject(e.what());
  }
  std::string reason;
  if (!source.empty() && !source_.empty() && source != source_)
    reason = "publisher restarted";
  else if (count_) {
    const auto previous = at(count_ - 1);
    double gap = received - *last_received_;
    if (std::abs(value.stamp - previous.stamp) <= 1e-6)
      return false;
    if (gap >= options_.stall)
      reason = "stream resumed";
    else if (value.stamp < previous.stamp) {
      if (!rewind_ || value.stamp <= rewind_->stamp) {
        rewind_ = value;
        return reject("Out-of-order reference frame ignored");
      }
      auto first = *rewind_;
      new_segment("source clock restarted");
      append(first);
    } else if (value.stamp - previous.stamp >= options_.stall)
      reason = "source time gap";
    else {
      double joint_jump = 0;
      for (size_t j = 0; j < indices_.size(); ++j)
        joint_jump = std::max(joint_jump, std::abs(value.pose.joints[j] - previous.pose.joints[j]));
      if ((value.pose.position.head<2>() - previous.pose.position.head<2>()).norm() >
              options_.jump_distance ||
          angular_velocity(yaw_quat(previous.pose.quaternion), yaw_quat(value.pose.quaternion))
                  .norm() > options_.jump_yaw ||
          joint_jump > options_.jump_joint)
        reason = "reference pose jumped";
    }
  }
  if (!reason.empty())
    new_segment(reason);
  else if (!count_)
    new_segment("initial stream");
  rewind_.reset();
  if (!source.empty())
    source_ = source;
  append(value);
  return true;
}
Pose Reference::raw(double stamp) const {
  size_t lo = 0, hi = count_;
  while (lo < hi) {
    size_t mid = (lo + hi) / 2;
    if (at(mid).stamp <= stamp)
      lo = mid + 1;
    else
      hi = mid;
  }
  if (lo == 0)
    return at(0).pose;
  if (lo == count_)
    return at(count_ - 1).pose;
  const auto& a = at(lo - 1);
  const auto& b = at(lo);
  return mix(a.pose, b.pose, (stamp - a.stamp) / (b.stamp - a.stamp), indices_.size());
}
Pose Reference::transformed(double stamp) const {
  auto p = raw(stamp);
  p.position = rotate(*fix_, p.position) + translation_;
  p.quaternion = normalize(multiply(*fix_, p.quaternion));
  if (blend_start_) {
    double u = std::clamp((stamp - *blend_start_) / blend_time_, 0., 1.);
    p = mix(*blend_from_, p, u * u * u * (10 + u * (-15 + 6 * u)), indices_.size());
  }
  return p;
}
void Reference::hold() {
  stale = true;
  if (!last_) {
    status = "waiting";
    throw ReferenceNotReady("Waiting for timestamped GMT window");
  }
  if (status != "refilling") {
    if (status != "holding")
      recovery_since_.reset();
    status = "holding";
  }
  snapshot_.fill(*last_);
  snapshot_ready_ = true;
}
void Reference::sample(double now) {
  require(std::isfinite(now), "Invalid sampling time");
  if (!count_ || now - *last_received_ >= options_.stall) {
    hold();
    return;
  }
  double offset = *std::min_element(offsets_.begin(), offsets_.begin() + offset_count_);
  double center = now - offset - (4 * options_.dt + options_.jitter);
  if (cursor_)
    center = std::max(center, *cursor_);
  if (center < at(0).stamp - 1e-6 || center + 4 * options_.dt > at(count_ - 1).stamp + 1e-6) {
    hold();
    return;
  }
  if (last_ && (status == "holding" || status == "refilling") && options_.recovery > 0 &&
      (!recovery_since_ || *last_received_ - *recovery_since_ < options_.recovery - 1e-6)) {
    hold();
    return;
  }
  if (!fix_) {
    auto p = raw(center);
    Q4 heading = robot_heading_.value_or(last_ ? yaw_quat(last_->quaternion) : Q4(1, 0, 0, 0));
    fix_ = normalize(multiply(heading, conjugate(yaw_quat(p.quaternion))));
    V3 anchor = robot_position_.value_or(last_ ? last_->position : V3::Zero());
    translation_.setZero();
    translation_.head<2>() = anchor.head<2>() - rotate(*fix_, p.position).head<2>();
    blend_start_ = blend_from_ ? std::optional<double>(center) : std::nullopt;
  } else if (status == "holding") {
    blend_from_ = last_;
    blend_start_ = center;
    blend_time_ = starting_ ? options_.startup : options_.blend;
    last_segment_reason = "reference window recovered without realignment";
  }
  cursor_ = center;
  for (int i = -8; i <= 4; ++i)
    snapshot_[i + 8] = transformed(center + i * options_.dt);
  last_ = snapshot_[8];
  snapshot_ready_ = true;
  stale = false;
  bool blending = blend_start_ && center < *blend_start_ + blend_time_;
  status = blending ? (starting_ ? "starting" : "blending") : "tracking";
  if (!blending)
    starting_ = false;
}
const Pose& Reference::frame(int offset) const {
  if (!snapshot_ready_)
    throw ReferenceNotReady("Reference window not ready");
  require(offset >= -8 && offset <= 4, "Reference offset outside window");
  return snapshot_[offset + 8];
}
}  // namespace noetix::policy
