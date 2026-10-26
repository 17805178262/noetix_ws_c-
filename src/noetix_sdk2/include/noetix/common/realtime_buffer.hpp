#ifndef NOETIX_COMMON_REALTIME_BUFFER_HPP_
#define NOETIX_COMMON_REALTIME_BUFFER_HPP_

#include <array>
#include <atomic>
#include <cstddef>

namespace noetix {

/**
 * @brief 初始化后不再分配内存的 SPSC“最新值”缓冲区（三槽快照交换）。
 *
 * 一个写者和一个读者通过三个预分配槽位交换快照。
 * 读者会发布自己正在拷贝的槽位下标，写者因此永远不会覆盖正在被读的内存。
 * 写者从不等待读者。
 *
 * 必须在实时循环启动之前调用 initialize()。
 * T 可以包含 std::vector 成员，但这些 vector 的容量应当在初始值里预先分配好。
 * 对于固定大小的控制数据，这样可以避免在周期循环里发生 new/delete。
 *
 * @note 只支持“恰好一个逻辑写者 + 一个逻辑读者”（SPSC）。
 */
template <class T>
class RealtimeBuffer {
 public:
  RealtimeBuffer() = default;

  // 用初始值直接构造并完成初始化。
  explicit RealtimeBuffer(const T& initial) { initialize(initial); }

  /**
   * @brief 用初始值填充三个槽位，并重置状态。
   *
   * @param initial 初始快照；三个槽位都会被填成这个值。
   *
   * @pre  必须在实时线程启动前、单线程环境下调用。
   * @post 三个槽位内容一致，published_ = 0，reading_ = kNoSlot，
   *       initialized_ = true。
   *
   * @note 这里对 T 做三次拷贝赋值。如果 T 内含 std::vector，
   *       容量由 initial 的容量决定，之后 write() 复用这些容量，
   *       不再触发堆分配。
   */
  void initialize(const T& initial) {
    for (auto& slot : slots_) {
      slot = initial;
    }
    // 初始没有读者在拷贝任何槽位。
    reading_.store(kNoSlot, std::memory_order_relaxed);
    // 初始发布槽位 0，读者默认读 0 号槽。
    published_.store(0, std::memory_order_release);
    initialized_.store(true, std::memory_order_release);
  }

  /**
   * @brief 查询是否已经 initialize()。
   * @return true 已初始化，可安全 read/write。
   */
  bool initialized() const noexcept {
    return initialized_.load(std::memory_order_acquire);
  }

  /**
   * @brief 发布一份新快照，不阻塞读者。
   *
   * 写者选择一个既不是“当前已发布槽”、也不是“读者正在拷贝槽”的空闲槽位，
   * 把 value 拷进去，然后原子地把 published_ 指向该槽位。
   *
   * @param value 要发布的新快照。
   *
   * @return true  发布成功；
   * @return false 仅当 initialize() 尚未调用，或找不到可用槽位（理论上不会发生）。
   *
   * @pre  调用者是唯一写者。
   * @post 成功时 published_ 指向新槽位，读者下次 read() 会拿到这份数据。
   *
   * @note 写者只写空闲槽位，绝不覆盖 published_ 或 reading_ 指向的槽位，
   *       因此不需要等待读者，也不会与读者产生数据竞争。
   * @note slots_[target] = value 的写入通过 published_.store(release) 发布，
   *       读者用 published_.load(acquire) 读取，保证看到完整快照。
   */
  bool write(const T& value) {
    if (!initialized()) {
      return false;
    }

    // 当前已发布的槽位（读者可能正在读它）。
    const int current = published_.load(std::memory_order_acquire);
    // 读者正在拷贝的槽位（kNoSlot 表示没有）。
    const int reading = reading_.load(std::memory_order_acquire);

    // 从三个槽位里挑一个既不是 current 也不是 reading 的槽位作为写入目标。
    int target = kNoSlot;
    for (int i = 0; i < static_cast<int>(slots_.size()); ++i) {
      if (i != current && i != reading) {
        target = i;
        break;
      }
    }

    // 三个槽位 + 单个读者，必然存在一个既未发布也未被拷贝的槽位。
    // 走到这里说明状态异常（例如被多个写者并发调用），直接失败。
    if (target == kNoSlot) {
      return false;
    }

    // 写空闲槽位；此时读者不可能在读它。
    slots_[static_cast<std::size_t>(target)] = value;
    // 原子发布：读者之后 load(acquire) 能看到完整的新快照。
    published_.store(target, std::memory_order_release);
    return true;
  }

  /**
   * @brief 把最新完整快照拷贝到调用者拥有的存储中。
   *
   * 读取流程：
   *  1. 读取 published_ 得到候选槽位；
   *  2. 把 reading_ 设为该槽位，声明“我正在拷贝它”；
   *  3. 再次读取 published_，确认在步骤 1~2 之间写者没有换槽；
   *     若换了，说明候选槽位可能即将被写者复用，撤销声明并重试；
   *  4. 确认未换后，拷贝槽位内容，最后把 reading_ 复位为 kNoSlot。
   *
   * 重试只覆盖“观察到 published_ 到声明 reading_”之间的极短窗口。
   * 一旦声明成功，写者就会排除该槽位，直到拷贝完成。
   *
   * @param[out] out 输出参数；读取成功时填入最新完整快照。
   *
   * @return true  成功读到一份完整快照；
   * @return false 未初始化，或 published_ 为 kNoSlot（尚无数据）。
   *
   * @pre  调用者是唯一读者。
   * @post 成功时 out 是某一次 write() 发布的完整快照；
   *       reading_ 复位为 kNoSlot。
   *
   * @note read 是 const 的，但会修改 mutable 的 reading_，
   *       因为“声明正在读”属于读操作的内部状态，不改变逻辑上的缓冲区内容。
   * @note 写者不会等待读者；读者最多在极短窗口内重试，
   *       适合实时线程调用。
   */
  bool read(T& out) const {
    if (!initialized()) {
      return false;
    }

    for (;;) {
      // 候选槽位：当前发布的槽位。
      const int index = published_.load(std::memory_order_acquire);
      if (index == kNoSlot) {
        // 还没有任何数据被发布。
        return false;
      }

      // 声明“我正在拷贝 index 槽位”，写者会避开它。
      reading_.store(index, std::memory_order_release);

      // 二次检查：如果在声明之前写者已经换过槽位，
      // 那么 index 可能已不是最新，且写者可能即将复用它，需要重试。
      if (published_.load(std::memory_order_acquire) != index) {
        reading_.store(kNoSlot, std::memory_order_release);
        continue;
      }

      // 此时写者不会碰 index 槽位，可以安全拷贝。
      out = slots_[static_cast<std::size_t>(index)];
      // 拷贝完成，释放该槽位。
      reading_.store(kNoSlot, std::memory_order_release);
      return true;
    }
  }

 private:
  // 表示“没有槽位”的哨兵值。
  static constexpr int kNoSlot = -1;

  // 三个预分配槽位；initialize() 之后不再改变大小。
  std::array<T, 3> slots_{};
  // 当前已发布的槽位下标；写者更新，读者读取。
  std::atomic<int> published_{kNoSlot};
  // 读者正在拷贝的槽位下标；读者更新，写者读取。
  // mutable 允许 const 的 read() 修改它。
  mutable std::atomic<int> reading_{kNoSlot};
  // 是否已完成 initialize()。
  std::atomic<bool> initialized_{false};
};

}  // namespace noetix

#endif  // NOETIX_COMMON_REALTIME_BUFFER_HPP_