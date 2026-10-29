// 中文说明：GMT参考缓冲：当前帧可用时继续跟随，未来缺帧只局部补齐，不外推未知动作。
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
// 开始新源段：旧段继续播放；只给新段计算坐标衔接和姿态过渡。
void Reference::new_segment(const std::string& reason, const TimedPose& first) {
  ++segments;
  last_segment_reason = reason;
  // 发布端重启会改变源时间与当前播放时钟的对应关系。
  if (reason == "publisher restarted" || reason == "source clock restarted") {
    playback_anchor_.reset();
    playback_reanchoring_ = true;
  }
  rewind_.reset();
  if (count_ && last_) {
    const auto& tail = at(count_ - 1);
    segment_anchor_ = tail.pose;
    segment_rotation_ = normalize(multiply(yaw_quat(tail.pose.quaternion),
                                          conjugate(yaw_quat(first.pose.quaternion))));
    segment_translation_ = tail.pose.position - rotate(segment_rotation_, first.pose.position);
    segment_translation_.z() = 0;
    // 正常源时间沿用原帧间隔；仅在发布者/源时钟重启时重建时间起点。
    double interval = options_.dt;
    if (previous_input_ && reason != "publisher restarted" && first.stamp > previous_input_->stamp)
      interval = first.stamp - previous_input_->stamp;
    segment_start_ = tail.stamp + interval;
    stamp_shift_ = *segment_start_ - first.stamp;
    return;
  }
  head_ = count_ = offset_head_ = offset_count_ = 0;
  playback_anchor_.reset();
  playback_reanchoring_ = false;
  fix_.reset();
  cursor_.reset();
  future_end_.reset();
  translation_.setZero();
  blend_start_.reset();
  segment_start_.reset();
  segment_anchor_.reset();
  segment_rotation_ = Q4(1, 0, 0, 0);
  segment_translation_.setZero();
  stamp_shift_ = 0;
  blend_from_ = initial_;
  blend_time_ = starting_ ? options_.startup : options_.blend;
  recovery_stamp_.reset();
  status = "waiting";
}
// 接收帧只处理一次新段变换，环形缓冲中保存同一坐标系、同一时间轴的姿态。
void Reference::append_input(const TimedPose& input) {
  previous_input_ = input;
  auto value = input;
  value.stamp += stamp_shift_;
  if (segment_start_) {
    value.pose.position = rotate(segment_rotation_, input.pose.position) + segment_translation_;
    value.pose.quaternion = normalize(multiply(segment_rotation_, input.pose.quaternion));
    double u = std::clamp((value.stamp - *segment_start_) / options_.blend, 0., 1.);
    double alpha = u * u * u * (10 + u * (-15 + 6 * u));
    if (u < 1.)
      value.pose = mix(*segment_anchor_, value.pose, alpha, indices_.size());
  }
  append(value);
}
void Reference::append(const TimedPose& value) {
  // 连续参考的跨度按源时间判断，集中到达的连续帧无需再等待80ms本机时间。
  if (status == "holding" || status == "refilling")
    if (!recovery_stamp_ || !count_ ||
        value.stamp - at(count_ - 1).stamp > std::max(.1, 4 * options_.dt))
      recovery_stamp_ = value.stamp;
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
  else if (previous_input_) {
    const auto previous = *previous_input_;
    if (std::abs(value.stamp - previous.stamp) <= 1e-6)
      return false;
    // 到达节奏只影响超时；帧顺序和缺口由消息内的源时间戳判断。
    if (value.stamp < previous.stamp) {
      if (!rewind_ || value.stamp <= rewind_->stamp) {
        rewind_ = value;
        return reject("Out-of-order reference frame ignored");
      }
      auto first = *rewind_;
      new_segment("source clock restarted", first);
      append_input(first);
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
    new_segment(reason, value);
  else if (!count_)
    new_segment("initial stream", value);
  rewind_.reset();
  if (!source.empty())
    source_ = source;
  append_input(value);
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
  return p;
}
void Reference::hold() {
  future_end_.reset();
  stale = true;
  if (!last_) {
    status = "waiting";
    throw ReferenceNotReady("Waiting for timestamped GMT window");
  }
  if (status != "refilling") {
    if (status != "holding")
      recovery_stamp_.reset();
    status = "holding";
  }
  snapshot_.fill(*last_);
  snapshot_ready_ = true;
}
// 单独设置播放时钟，避免把ROS时间和单调接收时间直接相减。
void Reference::set_playback_time(double time) {
  require(std::isfinite(time), "Invalid reference playback time");
  if (playback_time_ && time < *playback_time_ - 1e-6) {
    playback_anchor_.reset();
    playback_reanchoring_ = true;
  }
  playback_time_ = time;
}
void Reference::sample(double now) {
  require(std::isfinite(now), "Invalid sampling time");
  if (!count_ || now - *last_received_ >= options_.stall) {
    hold();
    return;
  }
  double offset = *std::min_element(offsets_.begin(), offsets_.begin() + offset_count_);
  const double live_center = now - offset - (4 * options_.dt + options_.jitter);
  double desired = live_center;
  bool reanchor = playback_reanchoring_;
  const char* reanchor_reason = "simulation clock restarted";
  if (playback_time_ && playback_anchor_) {
    desired = (*playback_anchor_)[1] + (*playback_time_ - (*playback_anchor_)[0]);
    // 长期慢仿真或暂停可能超过固定缓存容量；过渡到可用窗口，避免永久holding。
    if (desired < at(0).stamp - 1e-6 && last_) {
      desired = live_center;
      reanchor = true;
      reanchor_reason = "simulation reference backlog exceeded buffer";
    }
  }
  const bool recovering = last_ && (status == "holding" || status == "refilling");
  if (recovering && options_.recovery > 0 &&
      (!recovery_stamp_ || at(count_ - 1).stamp - *recovery_stamp_ < options_.recovery - 1e-6)) {
    hold();
    return;
  }
  // 以源时间轴正常播放，不根据缓冲余量减速或变速；缺少当前参考时直接保持。
  double center = cursor_ ? std::max(desired, *cursor_) : desired;
  if (center < at(0).stamp - 1e-6 || center > at(count_ - 1).stamp + 1e-6) {
    hold();
    return;
  }
  if (playback_time_ && (!playback_anchor_ || reanchor))
    playback_anchor_ = std::array<double, 2>{*playback_time_, center};
  if (!fix_) {
    auto p = raw(center);
    Q4 heading = robot_heading_.value_or(last_ ? yaw_quat(last_->quaternion) : Q4(1, 0, 0, 0));
    fix_ = normalize(multiply(heading, conjugate(yaw_quat(p.quaternion))));
    V3 anchor = robot_position_.value_or(last_ ? last_->position : V3::Zero());
    translation_.setZero();
    translation_.head<2>() = anchor.head<2>() - rotate(*fix_, p.position).head<2>();
    blend_start_ = blend_from_ ? std::optional<double>(center) : std::nullopt;
  } else if (recovering) {
    blend_from_ = last_;
    blend_start_ = center;
    blend_time_ = starting_ ? options_.startup : options_.blend;
    if (!segment_start_)
      last_segment_reason = "reference window recovered without realignment";
  }
  if (reanchor && last_) {
    blend_from_ = last_;
    blend_start_ = center;
    blend_time_ = starting_ ? options_.startup : options_.blend;
    future_end_.reset();
    last_segment_reason = reanchor_reason;
  }
  playback_reanchoring_ = false;
  // 没有的未来参考复用边界姿态。数据恢复时逐步补齐时间边界，
  // 让未来窗口连续接上新轨迹；当前帧和历史帧继续正常推进。
  const double available_end = std::min(center + 4 * options_.dt, at(count_ - 1).stamp);
  if (future_end_ && cursor_) {
    const double elapsed = std::max(0., center - *cursor_);
    const double normal_end = *future_end_ + elapsed;
    const double weight = std::clamp(elapsed / options_.blend, 0., 1.);
    future_end_ = std::min(available_end,
        normal_end + std::max(0., available_end - normal_end) * weight);
  } else {
    future_end_ = available_end;
  }
  cursor_ = center;

  // 整个窗口使用同一个过渡进度。恢复的首个窗口与保持姿态完全一致，
  // 避免中心帧未动、未来帧却提前跳到新动作；不重置本体和动作历史。
  const double u = blend_start_ ? std::clamp((center - *blend_start_) / blend_time_, 0., 1.) : 1.;
  const double alpha = u * u * u * (10 + u * (-15 + 6 * u));
  for (int i = -8; i <= 4; ++i) {
    auto pose = transformed(std::min(center + i * options_.dt, *future_end_));
    snapshot_[i + 8] = blend_from_ && u < 1.
        ? mix(*blend_from_, pose, alpha, indices_.size()) : pose;
  }
  last_ = snapshot_[8];
  snapshot_ready_ = true;
  stale = false;
  const bool blending = blend_start_ && u < 1.;
  const bool segment_blending = segment_start_ && center >= *segment_start_ &&
                                center < *segment_start_ + options_.blend;
  status = blending ? (starting_ ? "starting" : "blending")
                    : (segment_blending ? "blending" : "tracking");
  if (!blending) {
    starting_ = false;
    blend_start_.reset();
  }
}
const Pose& Reference::frame(int offset) const {
  if (!snapshot_ready_)
    throw ReferenceNotReady("Reference window not ready");
  require(offset >= -8 && offset <= 4, "Reference offset outside window");
  return snapshot_[offset + 8];
}
}  // namespace noetix::policy
