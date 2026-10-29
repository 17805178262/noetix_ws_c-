// 公共策略数学与历史缓冲；各策略的参数和观测入口位于 src/policies。
//
// 本文件主要包含：
// 1. IPolicy：所有策略的基类，负责 ONNX 输入缓冲区初始化、状态合法性校验、
//    动作解码与缩放；
// 2. TrackingPolicy：跟踪类策略，负责维护状态历史、动作历史，并构造
//    policy observation 与 frame command 两类输入；
// 3. estimate_root_position：当状态中没有根位置时，利用双足腿部正运动学
//    估计根高度。
#include "noetix_policy_cpp/core.hpp"

namespace noetix::policy {
namespace {
// 历史滞后采样表：表示从当前时刻往回取第 0、1、2、3、4、8、16 帧。
// 这些滞后量用于构造策略观测中的历史信息，使策略能够感知短时和较长时的动态变化。
constexpr std::array<int, 7> lags{0, 1, 2, 3, 4, 8, 16};
}  // namespace

// IPolicy 构造函数：根据配置中的 ONNX 输入规格，为每个输入分配连续浮点缓冲区。
// config.inputs 描述模型输入张量的形状；这里将每个输入按 shape 展平成一维缓冲区。
IPolicy::IPolicy(Spec value) : config(std::move(value)) {
  // inputs 是每个 ONNX 输入的连续存储缓冲区；先预留输入个数，避免反复扩容。
  inputs.reserve(config.inputs.size());

  // 遍历每个输入规格，计算展平后的元素个数，并用 0 初始化缓冲区。
  for (const auto& input : config.inputs) {
    size_t size = 1;

    // 将输入张量的各维大小相乘，得到展平后的元素总数。
    for (auto dim : input.shape) {
      require(dim > 0, "Invalid ONNX input shape");
      size *= size_t(dim);
    }

    // 每个输入都创建为 size 个 float，并初始化为 0.f。
    inputs.emplace_back(size, 0.f);
  }
}

// 校验机器人状态是否合法：
// - 四元数、陀螺仪、重力向量必须全部有限；
// - 每个关节的位置 q 和速度 dq 必须是有限值。
void IPolicy::validate_state(const State& state) const {
  require(state.quat.allFinite() && state.gyro.allFinite() && state.gravity.allFinite(),
          "Invalid state");

  for (int j = 0; j < config.dof(); ++j)
    require(std::isfinite(state.q[j]) && std::isfinite(state.dq[j]), "Invalid joint state");
}

// 将策略输出的原始动作解码为关节目标位置。
//
// 处理流程：
// 1. 检查动作指针非空，且动作数量等于自由度；
// 2. 先整体校验所有动作值均为有限值，避免部分写入后才发现 NaN/Inf；
// 3. 对动作做可选裁剪；
// 4. 校验由默认关节位置 + 缩放动作得到的目标位置也是有限值；
// 5. 写入 last_action 与 target。
//
// clip_walk 为 true 时，将动作裁剪到 [-18, 18]，用于行走类策略的限制。
void IPolicy::decode_scaled_action(const float* action, size_t count, bool clip_walk) {
  require(action != nullptr && count == size_t(config.dof()), "Wrong action count");

  // 在修改任何控制器状态之前，先校验整个动作数组，保证失败时不会留下半更新状态。
  for (size_t j = 0; j < count; ++j)
    require(std::isfinite(action[j]), "Action contains NaN/Inf");

  // 预检查缩放后的目标关节位置是否有限。
  for (size_t j = 0; j < count; ++j) {
    float a = clip_walk ? std::clamp(action[j], -18.f, 18.f) : action[j];
    require(std::isfinite(config.default_q[j] + a * config.action_scale[j]),
            "Target contains NaN/Inf");
  }

  // 正式写入动作历史与目标关节位置。
  // target = default_q + action * action_scale。
  for (size_t j = 0; j < count; ++j) {
    last_action[j] = clip_walk ? std::clamp(action[j], -18.f, 18.f) : action[j];
    target[j] = config.default_q[j] + last_action[j] * config.action_scale[j];
  }
}

// TrackingPolicy 构造函数：
// 仅允许 gmt 或 motion 类型，并且必须恰好有两个 ONNX 输入：
// - inputs[0]：frame command 输入；
// - inputs[1]：policy observation 输入。
// 构造时清空历史缓冲。
TrackingPolicy::TrackingPolicy(Spec config) : IPolicy(std::move(config)) {
  require((this->config.kind == Kind::gmt || this->config.kind == Kind::motion) &&
              inputs.size() == 2,
          "Invalid tracking policy contract");
  reset_history();
}

// 重置历史缓冲：
// - ready_ 表示状态历史是否已经完成首次填充；
// - history_head_ 是状态历史环形缓冲区当前写位置；
// - action_head_ 是动作历史环形缓冲区当前写位置；
// - last_action 与所有动作历史清零。
void TrackingPolicy::reset_history() {
  ready_ = false;
  history_head_ = 0;
  action_head_ = 0;
  last_action.fill(0);

  for (auto& history : action_history_)
    history.fill(0);
}

// 解码跟踪策略输出动作，并更新动作历史环形缓冲区。
// 跟踪策略不做 clip_walk 裁剪，因此第三个参数为 false。
void TrackingPolicy::decode(const float* action, size_t count) {
  decode_scaled_action(action, count, false);

  // 环形缓冲区写指针前移，并保存最新动作。
  action_head_ = (action_head_ + 1) % action_history_.size();
  action_history_[action_head_] = last_action;
}

// 构造 TrackingPolicy 的两类输入：
// 1. inputs[1]：policy observation，包含状态历史、动作历史等；
// 2. inputs[0]：frame command，包含 8 个 frame 的相对位置、相对姿态和关节角。
//
// 参数：
// - state：当前机器人状态；
// - frames：8 个参考 frame 的位姿与关节角；
// - center：中心点，用于把 frame 位置转换为相对中心的局部坐标；
// - heading：当前朝向，用于把世界坐标差旋转到 heading 局部坐标。
void TrackingPolicy::build_tracking(const State& state, const std::array<Pose, 8>& frames,
                                    const Pose& center, const F4& heading) {
  // 首次调用时，用当前状态填满整个状态历史，避免历史中出现未初始化数据。
  // 后续调用则正常推进环形缓冲区，并写入当前状态。
  if (!ready_) {
    history_.fill(state);
    history_head_ = 0;
    ready_ = true;
  } else {
    history_head_ = (history_head_ + 1) % history_.size();
    history_[history_head_] = state;
  }

  // k 是写入 policy observation 缓冲区的游标。
  size_t k = 0;

  // policy 指向 inputs[1]，即策略观测输入。
  auto& policy = inputs[1];

  // 构造状态历史项：
  // term = 0：gyro；
  // term = 1：gravity；
  // term = 2：关节位置 q；
  // term = 3：关节速度 dq。
  // 每个 term 都按照 lags 中的 7 个滞后量采样历史状态。
  for (int term = 0; term < 4; ++term)
    for (int lag : lags) {
      // 从环形缓冲区中取出“当前时刻往回 lag 帧”的状态。
      // 使用 history_.size() - lag 再取模，保证索引非负并正确回绕。
      const auto& h = history_[(history_head_ + history_.size() - size_t(lag)) % history_.size()];

      if (term < 2) {
        // term 0/1：写入 3 维 gyro 或 gravity。
        const auto& v = term == 0 ? h.gyro : h.gravity;
        for (float x : v)
          policy[k++] = x;
      } else {
        // term 2/3：写入所有关节的 q 或 dq。
        const auto& v = term == 2 ? h.q : h.dq;
        for (int j = 0; j < config.dof(); ++j)
          policy[k++] = v[j];
      }
    }

  // 构造动作历史项：取最近 3 帧动作，每帧包含所有关节维度。
  // lag = 0 表示当前动作，lag = 1 表示上一帧动作，lag = 2 表示上上帧动作。
  for (int lag = 0; lag < 3; ++lag) {
    const auto& a = action_history_[(action_head_ + action_history_.size() - size_t(lag)) %
                                    action_history_.size()];
    for (int j = 0; j < config.dof(); ++j)
      policy[k++] = a[j];
  }

  // 检查写入长度是否与 policy observation 缓冲区大小一致。
  require(k == policy.size(), "Internal policy shape mismatch");

  // 计算水平锚点：使用 center.position，但将 z 置 0。
  // 这样 frame 的位置输入只关注水平相对位置，不把中心高度混入水平平移。
  F3 anchor = center.position.cast<float>();
  anchor[2] = 0;

  // output 指向 inputs[0]，即 frame command 输入。
  auto& output = inputs[0];
  k = 0;

  // 构造 8 个 frame 的相对位置：
  // 1. 用 frame.position - anchor 得到相对锚点的位置；
  // 2. 用 conjugate(heading) 旋转到 heading 局部坐标；
  // 3. 每个 frame 写入 3 个 float。
  for (const auto& frame : frames) {
    F3 local = rotate(conjugate(heading), F3(frame.position.cast<float>() - anchor));
    for (float x : local)
      output[k++] = x;
  }

  // 构造 8 个 frame 的相对姿态：
  // q = normalize(conjugate(state.quat) * frame.quaternion)
  // 即当前机体姿态的共轭乘以 frame 姿态，得到 frame 相对当前机体的旋转。
  // 然后将该四元数转换为旋转矩阵的前两行，共 6 个元素写入输出。
  for (const auto& frame : frames) {
    F4 q = normalize(multiply(conjugate(state.quat), F4(frame.quaternion.cast<float>())));
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // 这里写入的是旋转矩阵的前两行：
    // [1-2(y^2+z^2), 2(xy-zw),     2(xz+yw)    ]
    // [2(xy+zw),     1-2(x^2+z^2), 2(yz-xw)    ]
    // 第三行省略，因此每个 frame 写入 6 个 float。
    for (float v : {1.f - 2.f * (y * y + z * z), 2.f * (x * y - z * w),
                    2.f * (x * z + y * w), 2.f * (x * y + z * w),
                    1.f - 2.f * (x * x + z * z), 2.f * (y * z - x * w)})
      output[k++] = v;
  }

  // 构造 8 个 frame 的关节角：每个 frame 写入所有自由度关节角。
  for (const auto& frame : frames)
    for (int j = 0; j < config.dof(); ++j)
      output[k++] = float(frame.joints[j]);

  // 检查写入长度是否与 frame command 缓冲区大小一致。
  require(k == output.size(), "Internal command shape mismatch");
}

// 估计根位置。
//
// 如果 state.position 已提供，则直接返回该位置。
// 否则，通过双足腿部正运动学估计根高度：
// - 根据N3 V3左右腿各6个关节角，累积计算踝关节位置；
// - 利用重力方向up与脚底球形支撑点，计算脚底的最低位置；
// - 取左右腿估计高度中的最大值作为根高度；
// - 水平位置无法通过该函数恢复，因此返回 (0, 0, height)。
V3 estimate_root_position(const State& state) {
  // 如果外部已经提供根位置，直接使用。
  if (state.position)
    return *state.position;

  // 重力方向取反得到“向上”方向。
  V3 up = -state.gravity.cast<double>();
  up.normalize();

  // height 用于保存左右腿估计出的最大根高度。
  double height = -1e9;

  // axes 表示每条腿 6 个关节的旋转轴。
  // 左右腿共用同一组关节轴方向定义。
  const std::array<V3, 6> axes{V3(0, 1, 0), V3(1, 0, 0), V3(0, 0, 1),
                               V3(0, 1, 0), V3(0, 1, 0), V3(1, 0, 0)};

  // side = 1 表示左腿，side = -1 表示右腿。
  for (int side : {1, -1}) {
    // 每条腿在 state.q 中的起始索引：左腿从 0 开始，右腿从 6 开始。
    int start = side == 1 ? 0 : 6;

    // offsets 是每条腿 6 个连杆的局部平移偏移。
    // 这些偏移按关节链依次累加，用于正运动学计算脚部中心。
    std::array<V3, 6> offsets{V3(.000056967, side * .125, .059553), V3(-.0001, 0, -.09),
                              V3(0, 0, -.135), V3(.0001, 0, -.1551),
                              V3(0, -side * .00037979, -.34), V3::Zero()};

    // rotation 表示当前累积的连杆旋转矩阵，初始为单位阵。
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    // position 表示当前累积的脚部中心位置，初始为 0。
    V3 position = V3::Zero();

    // 遍历该腿的 6 个关节，逐级进行正运动学累积。
    for (int j = 0; j < 6; ++j) {
      // 先加上当前连杆的局部平移偏移，并按当前累积旋转变换到父坐标系。
      position += rotation * offsets[j];

      // 当前关节旋转轴。
      const auto& a = axes[j];

      // 构造旋转轴 a 的反对称矩阵 skew，用于 Rodrigues 公式。
      Eigen::Matrix3d skew;
      skew << 0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0;

      // 当前关节角。
      float angle = state.q[start + j];

      // 使用 Rodrigues 公式计算当前关节旋转：
      // R = I + sin(angle) * skew + (1 - cos(angle)) * skew^2
      // 然后右乘到累积旋转上。
      rotation = rotation * (Eigen::Matrix3d::Identity() + double(std::sin(angle)) * skew +
                             (1 - double(std::cos(angle))) * (skew * skew));
    }

    // 脚底12个半径5mm的球形支撑点与N3 V3部署URDF一致，不再使用旧版脚底包围盒。
    const std::array<V3, 12> support{
        V3(-.060, 0, -.060), V3(-.055, -.025, -.060), V3(-.055, .025, -.060),
        V3(-.040, -.040, -.060), V3(-.040, .040, -.060),
        V3(.010, -.043, -.060), V3(.010, .043, -.060),
        V3(.075, -.050, -.060), V3(.075, .050, -.060),
        V3(.140, -.035, -.0595), V3(.140, .035, -.0595), V3(.168, 0, -.052)};
    for (const auto& point : support)
      height = std::max(height, -up.dot(position + rotation * point) + .005);
  }

  // 该估计只能恢复竖直高度，水平位置未知，因此返回 (0, 0, height)。
  return {0, 0, height};
}
}  // namespace noetix::policy