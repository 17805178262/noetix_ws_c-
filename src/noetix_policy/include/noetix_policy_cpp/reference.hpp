// 中文说明：GMT带时间戳参考缓冲的C++实现，使用pose_only参考窗口，缺少未来帧时局部补齐。
// 保留乱序丢弃、连续时间回退判定、跳变分段、起步/重连五次平滑、缺包锁存和水平重对齐。
// 存储固定512帧、64个时钟偏移和13帧快照；接收与采样的互斥由ROS节点管理。
// 所有时间由调用方传入，方便用相同事件流对照Python，不使用网络到包数量代替训练时间。
#pragma once
#include "noetix_policy_cpp/types.hpp"
namespace noetix::policy {
struct ReferenceOptions {
  // 正常参考延后600ms：520ms抖动缓冲 + 未来4帧的80ms；训练采样间隔仍为20ms。
  // 1秒断流保护与播放延迟分别设置，恢复连续性按源帧时间检查。
  double dt = .02, jitter = .52, stall = 1., blend = .5, startup = 2., recovery = .08;
  double jump_distance = .35, jump_yaw = 3.141592653589793 / 4, jump_joint = .8;
};
class Reference {
 public:
  explicit Reference(std::vector<int> indices, ReferenceOptions options = {});
  bool push(const std::vector<double>& values, double received, const std::string& source = "");
  void align(const Q4& quaternion, const std::optional<V3>& position = {});
  void seed(const Pose& pose);
  // 仿真在每次推理前传入ROS时间；未设置时，真机继续使用sample的单调时钟。
  void set_playback_time(double time);
  void sample(double now);
  const Pose& frame(int offset = 0) const;
  std::string status = "waiting", last_rejection, last_segment_reason = "initial stream";
  size_t rejected = 0, segments = 0;
  bool stale = false;

 private:
  struct TimedPose {
    // stamp为GMR消息携带的输入帧时间；received为GMT本机接收时间，仅用于对时和超时。
    double stamp = 0, received = 0;
    Pose pose;
  };
  std::array<TimedPose, 512> frames_{};
  size_t head_ = 0, count_ = 0, offset_head_ = 0, offset_count_ = 0;
  std::array<double, 64> offsets_{};
  std::array<Pose, 13> snapshot_{};
  bool snapshot_ready_ = false, starting_ = false;
  std::vector<int> indices_;
  ReferenceOptions options_;
  std::string source_;
  std::optional<TimedPose> rewind_, previous_input_;
  // 新源段先转换到旧参考坐标系，已缓存的旧段不重新变换。
  Q4 segment_rotation_{1, 0, 0, 0};
  V3 segment_translation_ = V3::Zero();
  std::optional<Pose> segment_anchor_;
  std::optional<double> segment_start_;
  double stamp_shift_ = 0;
  std::optional<Q4> fix_, robot_heading_;
  V3 translation_ = V3::Zero();
  std::optional<V3> robot_position_, position_offset_;
  std::optional<Pose> initial_, last_, blend_from_;
  std::optional<double> cursor_, blend_start_, last_received_, recovery_stamp_;
  // 未来参考的可用采样边界；缺帧时夹取最近有效姿态，恢复后平滑推进。
  std::optional<double> future_end_;
  double blend_time_ = 0;
  // {仿真时钟锚点, 对应源时间}；接收超时始终使用本机单调时间。
  std::optional<double> playback_time_;
  std::optional<std::array<double, 2>> playback_anchor_;
  bool playback_reanchoring_ = false;
  const TimedPose& at(size_t i) const { return frames_[(head_ + i) % frames_.size()]; }
  bool reject(const std::string& reason);
  void append(const TimedPose& frame);
  void new_segment(const std::string& reason, const TimedPose& first);
  void append_input(const TimedPose& frame);
  Pose raw(double stamp) const;
  Pose transformed(double stamp) const;
  void hold();
};
}  // namespace noetix::policy
