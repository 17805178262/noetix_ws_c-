/**
 * @file io_loop.cpp
 * @brief IoLoop 的实现：周期性读取机器人状态并下发 MIT 命令。
 *
 * 线程模型：
 *  - 周期线程：由 io_thread_ 驱动，每个周期调用 cycle()，负责 read/write。
 *  - 外部线程：调用 setCommand()/getLatestState() 等接口，通过双缓冲与周期线程交换数据。
 *  - 控制面：enableDrive()/disableDrive() 与 cycle() 通过 robot_mutex_ 互斥。
 *
 * 数据流：
 *  - 状态：robot_->read() -> state_ -> state_buffer_.write() -> 外部 getLatestState()。
 *  - 命令：外部 setCommand() -> command_buffer_.write() -> cycle() 读取 -> robot_->write()。
 */

#include "noetix_bridge/io_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace noetix_bridge {

namespace {
/**
 * @brief 获取 steady_clock 当前时间的纳秒计数。
 *
 * 使用 steady_clock 而非 system_clock，保证时间单调递增，
 * 不受系统时间调整（NTP、手动改时）影响，适合做周期控制的时间戳。
 *
 * @return 自 steady_clock 纪元起的纳秒数，类型为 uint64_t。
 * @note 仅用于相对时间戳，不代表墙钟时间。
 */
std::uint64_t steadyNowNs()
{
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

/**
 * @brief 构造 IoLoop。
 *
 * 完成以下初始化：
 *  1. 保存 robot 接口与执行器名称列表；
 *  2. 建立 name -> index 的索引，用于 setCommand 按名称重排；
 *  3. 初始化状态容器 state_，并填入每个电机的名称；
 *  4. 初始化 zero_command_ 与 command_snapshot_ 并全部置零；
 *  5. 初始化 state_buffer_ 与 command_buffer_ 两个双缓冲。
 *
 * @param robot          机器人硬件接口，允许为空（为空时 start 仍可成功，
 *                       但 cycle 会直接返回，enableDrive 会失败）。
 * @param actuator_names 执行器名称列表，顺序即“规范顺序”，
 *                       后续所有命令都会按此顺序重排。
 *
 * @pre  actuator_names 中的名称应唯一；若重复，actuator_index_.emplace
 *       只保留第一个，setCommand 对重复名称的检测仍能生效。
 * @post dof_ == actuator_names_.size()，所有内部缓冲区大小均为 dof_。
 *
 * @note 成员初始化列表中 dof_(actuator_names_.size()) 依赖类中
 *       actuator_names_ 声明在 dof_ 之前；否则 dof_ 可能先于
 *       actuator_names_ 初始化而得到 0。实际初始化顺序由声明顺序决定。
 */
IoLoop::IoLoop(std::shared_ptr<noetix::RobotInterface> robot,
               std::vector<std::string> actuator_names)
: robot_(std::move(robot)),                       // 接管机器人接口所有权
  actuator_names_(std::move(actuator_names)),     // 接管执行器名称列表
  dof_(actuator_names_.size())                    // 自由度 = 执行器数量
{
  // 建立“执行器名称 -> 下标”的索引，供 setCommand 按名称重排命令使用。
  // reserve 避免循环中反复扩容。
  actuator_index_.reserve(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    actuator_index_.emplace(actuator_names_[i], i);
  }

  // 初始化状态容器，并把每个电机状态的名字填成规范顺序。
  state_.resize(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    state_.motor_state[i].name = actuator_names_[i];
  }

  // 初始化零命令与命令快照：先 resize，再填名字，最后全部置零。
  zero_command_.resize(dof_);
  command_snapshot_.resize(dof_);
  damping_command_.resize(dof_);
  initializeCommand(damping_command_);
  zeroCommand(damping_command_);
  initializeCommand(zero_command_);   // 设置大小与每个电机名字
  initializeCommand(command_snapshot_);
  zeroCommand(zero_command_);         // q/dq/tau/kp/kd 全置 0
  zeroCommand(command_snapshot_);

  // 初始化双缓冲。initialize 会建立内部的两块缓冲区并同步当前值。
  state_buffer_.initialize(state_);
  command_buffer_.initialize(zero_command_);
}

/**
 * @brief 析构 IoLoop。
 *
 * 调用 stop() 确保周期线程已停止，避免线程在对象销毁后继续访问成员。
 *
 * @note 析构函数不抛出异常；stop() 内部也不应抛异常。
 */
IoLoop::~IoLoop()
{
  stop();
}

/**
 * @brief 确保 LowCmd 的大小为 dof_，并按规范顺序填入每个电机的名称。
 *
 * @param[in,out] cmd 待初始化的命令对象。
 *
 * @post cmd.motor_cmd.size() == dof_；
 *       cmd.motor_cmd[i].name == actuator_names_[i]。
 *
 * @note 仅修改大小和名称，不修改 q/dq/tau/kp/kd 等控制量。
 */
void IoLoop::initializeCommand(noetix::LowCmd& cmd) const
{
  if (cmd.motor_cmd.size() != dof_) {
    cmd.resize(dof_);
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    cmd.motor_cmd[i].name = actuator_names_[i];
  }
}

/**
 * @brief 把命令中所有电机的 MIT 控制量清零。
 *
 * 将每个 motor_cmd 的 q、dq、tau、kp、kd 全部置为 0.0F。
 *
 * @param[in,out] cmd 待清零的命令对象。
 *
 * @post 所有控制量为 0；名称、sequence、timestamp_ns 保持不变。
 *
 * @note 该函数为 noexcept，可在 disableDrive 等关键路径安全调用。
 *       用于初始化 zero_command_，以及 disable 前发送一帧零命令。
 */
void IoLoop::zeroCommand(noetix::LowCmd& cmd) const noexcept
{
  for (auto& c : cmd.motor_cmd) {
    c.q = 0.0F;
    c.dq = 0.0F;
    c.tau = 0.0F;
    c.kp = 0.0F;
    c.kd = 0.0F;
  }
}

/**
 * @brief 检查命令中所有电机控制量是否均为有限值。
 *
 * 对每个电机的 q、dq、tau、kp、kd 调用 std::isfinite，
 * 任一为 NaN 或 Inf 即返回 false。
 *
 * @param[in] cmd 待检查的命令对象。
 * @return true  所有控制量均有限；
 * @return false 存在 NaN 或 Inf。
 *
 * @note 用于 setCommand 下发前的最后一道数值合法性校验，
 *       防止非法值被写入硬件命令缓冲。
 */
bool IoLoop::finiteCommand(const noetix::LowCmd& cmd) const noexcept
{
  for (const auto& c : cmd.motor_cmd) {
    if (!std::isfinite(c.q) || !std::isfinite(c.dq) ||
        !std::isfinite(c.tau) || !std::isfinite(c.kp) ||
        !std::isfinite(c.kd)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 启动周期线程。
 *
 * 幂等语义：若线程已在运行，直接返回 true，不会重复创建线程。
 * 启动流程：
 *  1. 用 running_.exchange(true) 原子地“抢占”启动权；
 *  2. 校验频率合法性，非法则回滚 running_ 并返回 false；
 *  3. 频率换算为纳秒周期；
 *  4. 调用 io_thread_.start() 启动线程，失败则回滚 running_ 并返回 false。
 *
 * @param frequency 目标运行频率，单位 Hz，必须 > 0。
 * @param options   周期线程配置（调度策略、优先级、栈大小等），
 *                  原样透传给 io_thread_.start()。
 *
 * @return true  线程已在运行或本次启动成功；
 * @return false 频率非法或线程启动失败。
 *
 * @pre  frequency > 0 才会真正启动线程。
 * @post 成功时 running_ == true 且周期线程已启动；
 *       失败时 running_ 已回滚为 false。
 *
 * @note 使用 exchange 而非 load+store，保证“判断+置位”原子完成，
 *       避免并发调用 start() 时启动两个线程。
 * @note memory_order_acq_rel 提供获取-释放语义，保证状态同步正确。
 * @warning 周期由 1.0e9 / frequency 截断取整得到，高频时实际周期
 *          可能略小于理论值。
 */
bool IoLoop::start(double frequency,
                   const noetix::PeriodicThreadOptions& options)
{
  // 原子地把 running_ 置为 true，并返回旧值。
  // 旧值为 true 说明线程已在运行，直接返回，避免重复启动。
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return true;
  }
  // 频率非法时回滚 running_，返回失败。
  if (frequency <= 0.0) {
    running_.store(false, std::memory_order_release);
    return false;
  }

  // 频率 -> 周期（纳秒）。这里为截断取整。
  const auto period_ns = static_cast<std::uint64_t>(1.0e9 / frequency);
  
  // 启动周期线程，每周期回调 cycle()。启动失败同样回滚 running_。
  if (!io_thread_.start(
          std::chrono::nanoseconds(period_ns),
          [this]() { cycle(); },
          options)) {
    running_.store(false, std::memory_order_release);
    return false;
  }
  return true;
}

/**
 * @brief 停止周期线程。
 *
 * 用 running_.exchange(false) 原子地“释放”运行状态：
 *  - 若旧值为 false，说明本来就没运行，直接返回；
 *  - 若旧值为 true，置为 false 后调用 io_thread_.stop() 停止线程。
 *
 * @post 成功返回后 running_ == false 且周期线程已停止。
 *
 * @note 幂等：重复调用无副作用。
 * @note exchange 保证并发调用 stop() 时只有一个线程真正执行停止逻辑。
 */
void IoLoop::stop()
{
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  io_thread_.stop();
}

/**
 * @brief 设置一帧新的 MIT 命令。
 *
 * 处理流程：
 *  1. 检查驱动已使能，否则拒绝；
 *  2. 检查命令数量等于 dof_；
 *  3. 构造按规范顺序排列的 ordered 命令；
 *  4. 按名称把输入命令重排到 ordered 的对应位置，
 *     同时校验名称非空、存在、且不重复；
 *  5. 校验所有控制量有限；
 *  6. 写入 command_buffer_，成功后置 command_valid_ = true。
 *
 * @param[in] command 输入命令，motor_cmd 顺序不要求与规范顺序一致，
 *                    但每个元素的 name 必须合法且不重复。
 *
 * @return true  命令已成功写入命令缓冲；
 * @return false 驱动未使能、数量不符、名称非法/重复、数值非法，
 *               或命令缓冲写入失败。
 *
 * @pre  drive_enabled_ == true。
 * @pre  command.motor_cmd.size() == dof_。
 * @post 成功时 command_valid_ == true，下一周期 cycle() 会下发该命令；
 *       失败时命令缓冲与 command_valid_ 均不变。
 *
 * @note 该函数不直接调用 robot_->write()，只更新命令缓冲；
 *       真正的下发由周期线程 cycle() 完成，避免与周期线程竞争硬件。
 * @note 按名称重排而非按下标，允许调用方以任意顺序提交命令。
 */
bool IoLoop::setCommand(const noetix::LowCmd& command)
{
  // 失能和阻尼期间均拒绝上层命令，防止旧策略覆盖阻尼输出。
  if (!drive_enabled_.load(std::memory_order_acquire) || damping_.load(std::memory_order_acquire)) {
    return false;
  }
  // 命令数量必须与自由度一致。
  if (command.motor_cmd.size() != dof_) {
    return false;
  }

  // 构造一个按规范顺序排列的新命令。
  noetix::LowCmd ordered;
  ordered.resize(dof_);
  ordered.sequence = command.sequence;
  ordered.timestamp_ns = command.timestamp_ns;
  initializeCommand(ordered);  // 名称按 actuator_names_ 顺序填好

  // seen 用于检测重复的执行器名称。
  std::vector<std::uint8_t> seen(dof_, 0U);
  for (const auto& src : command.motor_cmd) {
    // 名称不能为空，且必须存在于索引中。
    const auto it = actuator_index_.find(src.name);
    if (src.name.empty() || it == actuator_index_.end()) {
      return false;
    }
    const std::size_t index = it->second;
    // 同一个执行器出现两次视为非法。
    if (seen[index] != 0U) {
      return false;
    }
    seen[index] = 1U;

    // 把输入命令按名称放到规范顺序对应的位置。
    auto& dst = ordered.motor_cmd[index];
    dst.q = src.q;
    dst.dq = src.dq;
    dst.tau = src.tau;
    dst.kp = src.kp;
    dst.kd = src.kd;
  }

  // 数值必须全部有限，防止 NaN/Inf 下发到硬件。
  if (!finiteCommand(ordered)) {
    return false;
  }

  // 写入命令双缓冲；写失败（例如缓冲区正被读）则返回失败。
  if (!command_buffer_.write(ordered)) {
    return false;
  }
  // 标记当前有有效命令，cycle() 会据此决定是否下发命令而非零命令。
  command_valid_.store(true, std::memory_order_release);
  return true;
}

/**
 * @brief 清除“有有效命令”的标记。
 *
 * 下一周期 cycle() 检测到 command_valid_ == false 后，
 * 会退回发送 zero_command_，实现命令失效时的零输出兜底。
 *
 * @post command_valid_ == false。
 *
 * @note noexcept；仅做一次原子 store，不涉及锁。
 * @note 该函数不修改 command_buffer_ 中的内容，只改变有效性标记。
 */
void IoLoop::clearCommand() noexcept
{
  command_valid_.store(false, std::memory_order_release);
}

/**
 * @brief 查询当前是否存在有效命令。
 *
 * @return true  存在有效命令；
 * @return false 无有效命令或已被 clearCommand 清除。
 *
 * @note 使用 acquire 语义，保证看到 true 时能观察到 setCommand
 *       在此之前对命令缓冲的写入。
 */
bool IoLoop::hasCommand() const noexcept
{
  return command_valid_.load(std::memory_order_acquire);
}

/**
 * @brief 使能驱动。
 *
 * 流程：
 *  1. 先清除旧命令，避免使能瞬间下发上一次的陈旧命令；
 *  2. 在 robot_mutex_ 保护下调用 robot_->enableActuators()；
 *  3. 只有接口返回成功且 actuatorsEnabled() 查询为 true，
 *     才认为使能成功并更新 drive_enabled_。
 *
 * @return true  使能成功；
 * @return false robot_ 为空，或使能失败/查询未使能。
 *
 * @pre  robot_ 非空才会真正调用厂商接口。
 * @post 成功时 drive_enabled_ == true；失败时 drive_enabled_ == false。
 *
 * @note 持 robot_mutex_ 与周期线程的 cycle() 互斥，
 *       避免 enable 期间周期线程并发 read/write。
 * @note “返回成功 + 查询确认为真”双重校验，防止厂商接口返回成功
 *       但硬件实际未使能的情况。
 */
bool IoLoop::enableDrive()
{
  clearCommand();
  std::lock_guard<std::mutex> lock(robot_mutex_);  // 保护 robot_ 的并发访问
  if (!robot_) {
    return false;
  }
  damping_.store(false, std::memory_order_release);
  const bool ok = robot_->enableActuators();
  // 只有厂商接口返回成功且查询确实为使能，才认为使能成功。
  const bool enabled = ok && robot_->actuatorsEnabled();
  drive_enabled_.store(enabled, std::memory_order_release);
  return enabled;
}

/**
 * @brief 禁用驱动。
 *
 * 流程：
 *  1. 清除有效命令并立即把 drive_enabled_ 置 false，
 *     使 setCommand 立即拒绝新命令、cycle 立即退回零命令；
 *  2. 在 robot_mutex_ 保护下，先显式写一帧零命令，
 *     再调用 robot_->disableActuators()；
 *  3. 只有禁用成功且查询确认为未使能，才返回 true。
 *
 * @return true  禁用成功；
 * @return false robot_ 为空，或禁用失败/查询仍为使能。
 *
 * @post drive_enabled_ == false。
 *
 * @note 在 disable 之前写一帧零命令，是为了保证硬件命令缓冲中
 *       不残留上一次的 MIT 增益（kp/kd），避免禁用瞬间产生残余力矩。
 *       这只是一个清理动作，不是安全控制器。
 * @note 持 robot_mutex_ 与 cycle() 互斥，避免零命令与周期命令交错下发。
 */
bool IoLoop::disableDrive()
{
  damping_.store(false, std::memory_order_release);
  clearCommand();
  drive_enabled_.store(false, std::memory_order_release);

  std::lock_guard<std::mutex> lock(robot_mutex_);
  if (!robot_) {
    return false;
  }

  // 在调用厂商 disable 之前显式写一帧零命令。
  // 这不是安全控制器，只是确保硬件命令缓冲里不残留上一次的 MIT 增益。
  robot_->write(zero_command_);
  const bool ok = robot_->disableActuators();
  // 只有禁用成功且查询确实为未使能，才返回 true。
  return ok && !robot_->actuatorsEnabled();
}

/**
 * @brief 查询驱动是否使能。
 *
 * @return true  驱动已使能；
 * @return false 驱动未使能。
 *
 * @note 使用 acquire 语义，与 enableDrive/disableDrive 中的 release 配对。
 */
// 只撤掉刚度和前馈，保留速度阻尼；不自动使能已关闭的驱动。
bool IoLoop::beginDamping(double kd)
{
  if (!std::isfinite(kd) || kd < 0.0) return false;
  std::lock_guard<std::mutex> lock(robot_mutex_);
  if (!robot_ || !drive_enabled_.load(std::memory_order_acquire)) return false;
  for (auto& motor : damping_command_.motor_cmd) motor.kd = static_cast<float>(kd);
  damping_.store(true, std::memory_order_release);
  clearCommand();
  return robot_->write(damping_command_);
}

// 查询阻尼阶段，驱动在此期间仍然使能。
bool IoLoop::dampingActive() const noexcept
{
  return damping_.load(std::memory_order_acquire);
}

// 与I/O周期互斥地发送人工清错请求，完全失能后才允许执行。
bool IoLoop::clearMotorErrors()
{
  std::lock_guard<std::mutex> lock(robot_mutex_);
  if (!robot_ || drive_enabled_.load(std::memory_order_acquire) || robot_->actuatorsEnabled())
    return false;
  return robot_->clearActuatorErrors();
}

bool IoLoop::driveEnabled() const noexcept
{
  return drive_enabled_.load(std::memory_order_acquire);
}

/**
 * @brief 读取最新一帧机器人状态。
 *
 * 从 state_buffer_ 双缓冲中读取周期线程写入的最新状态，
 * 不直接访问 state_，避免与周期线程竞争。
 *
 * @param[out] state 输出参数，读取成功时填入最新状态。
 *
 * @return true  成功读取到一帧状态；
 * @return false 读取失败（例如尚无新数据或缓冲区交换失败）。
 *
 * @note 该函数可在任意线程调用，不持有 robot_mutex_。
 */
bool IoLoop::getLatestState(noetix::LowState& state) const
{
  return state_buffer_.read(state);
}

/**
 * @brief 获取周期线程的统计信息。
 *
 * @return 包含周期抖动、超时次数、实际周期等信息的统计结构。
 *
 * @note 仅做只读查询，不阻塞周期线程。
 */
noetix::PeriodicThreadStats IoLoop::stats() const noexcept
{
  return io_thread_.stats();
}

/**
 * @brief 获取累计读错误次数。
 *
 * @return 自对象创建以来 robot_->read() 失败的总次数。
 *
 * @note 使用 relaxed 语义：只关心计数值本身，不承担同步职责。
 */
std::uint64_t IoLoop::readErrors() const noexcept
{
  return read_errors_.load(std::memory_order_acquire);
}

/**
 * @brief 获取累计写错误次数。
 *
 * @return 自对象创建以来 robot_->write() 失败的总次数。
 *
 * @note 使用 relaxed 语义：只关心计数值本身，不承担同步职责。
 */
std::uint64_t IoLoop::writeErrors() const noexcept
{
  return write_errors_.load(std::memory_order_acquire);
}

/**
 * @brief 周期回调，每个控制周期执行一次。
 *
 * 执行顺序：
 *  1. 加 robot_mutex_，保证 read/write 不与 enable/disable 并发；
 *  2. 读取状态：成功则打时间戳、递增序号并写入 state_buffer_；
 *     失败则只累加 read_errors_，state_ 保留上一次内容；
 *  3. 选择输出命令：默认 zero_command_；
 *     仅当 drive_enabled_ && command_valid_ 时读取 command_buffer_，
 *     使用 command_snapshot_ 作为输出；
 *  4. 下发命令：写失败只累加 write_errors_，不中断周期线程。
 *
 * @pre  由 io_thread_ 在周期线程中调用；不应被外部直接调用。
 * @post 本周期完成一次 read + 一次 write（robot_ 为空时除外）。
 *
 * @note 整个 cycle 持 robot_mutex_，因此 enableDrive/disableDrive
 *       会与周期线程互斥，避免硬件访问交错。
 * @note 读失败时保留上一次快照，写仍使用同一缓冲区，
 *       保证输出命令的连续性。
 * @note 任何 read/write 失败都不会抛出异常或终止线程，
 *       只通过错误计数反映，保证周期线程持续运行。
 */
void IoLoop::cycle()
{
  // 整个 cycle 持锁，保证 robot_ 的 read/write 不与 enable/disable 并发。
  std::lock_guard<std::mutex> lock(robot_mutex_);
  if (!robot_) {
    return;
  }

  // 1) 读取状态。
  if (robot_->read(state_)) {
    // 读取成功：打时间戳、递增序号，然后写入状态双缓冲供外部读取。
    state_.timestamp_ns = steadyNowNs();
    state_.sequence = ++state_sequence_;
    state_buffer_.write(state_);
  } else {
    // 读取失败：只累加错误计数，state_ 保持上一次内容。
    read_errors_.fetch_add(1U, std::memory_order_relaxed);
  }

  // 2) 选择本周期要下发的命令。
  // 默认输出零命令；只有驱动使能且存在有效命令时，才使用命令快照。
  const noetix::LowCmd* output = &zero_command_;
  if (damping_.load(std::memory_order_acquire) && drive_enabled_.load(std::memory_order_acquire)) {
    output = &damping_command_;
  } else if (drive_enabled_.load(std::memory_order_acquire) &&
      command_valid_.load(std::memory_order_acquire)) {
    // 从命令双缓冲读取最新命令到快照。
    // 读失败时保留上一次快照，因此无论成功与否都用这个缓冲区下发。
    (void)command_buffer_.read(command_snapshot_);
    output = &command_snapshot_;
  }

  // 3) 下发命令。写失败只累加错误计数，不中断周期线程。
  if (!robot_->write(*output)) {
    write_errors_.fetch_add(1U, std::memory_order_relaxed);
  }
}

}  // namespace noetix_bridge