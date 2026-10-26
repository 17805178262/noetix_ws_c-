#ifndef NOETIX_COMMON_PERIODIC_THREAD_HPP_
#define NOETIX_COMMON_PERIODIC_THREAD_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <functional>
#include <iostream>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#endif

namespace noetix {

struct PeriodicThreadOptions {
  bool realtime{true};
  bool lock_memory{true};
  bool strict_realtime{false};
  int priority{80};
  int cpu_affinity{-1};
};

struct PeriodicThreadStats {
  std::uint64_t cycles{0};
  std::uint64_t overruns{0};
  std::uint64_t consecutive_overruns{0};
  std::uint64_t max_exec_ns{0};
  std::uint64_t max_lateness_ns{0};
  std::uint64_t last_exec_ns{0};
  std::uint64_t last_lateness_ns{0};
  bool sched_fifo_active{false};
  bool memory_locked{false};
  int cpu_affinity{-1};
  int priority{0};
};

/**
 * @brief Absolute-time periodic worker with optional Linux RT setup.
 *
 * On Linux the thread can use SCHED_FIFO, CPU affinity and mlockall. The loop
 * sleeps against CLOCK_MONOTONIC absolute deadlines, avoiding accumulated
 * drift. RT setup is best-effort unless strict_realtime is enabled.
 */
class PeriodicThread {
 public:
  PeriodicThread() = default;
  ~PeriodicThread() { stop(); }

  PeriodicThread(const PeriodicThread&) = delete;
  PeriodicThread& operator=(const PeriodicThread&) = delete;

  bool start(std::chrono::nanoseconds period,
             std::function<void()> fn,
             const PeriodicThreadOptions& options = {}) 
  {
    stop();
    resetStats();

    if (period.count() <= 0 || !fn) {
      return false;
    }

    period_ = period;
    options_ = options;
    running_.store(true, std::memory_order_release);
    start_gate_.store(false, std::memory_order_release);

    thread_ = std::thread([this, fn = std::move(fn)]() mutable {
      while (!start_gate_.load(std::memory_order_acquire) &&
             running_.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      if (!running_.load(std::memory_order_acquire)) {
        return;
      }
      runLoop(fn);
    });

    bool setup_ok = configureRealtime();
    if (!setup_ok && options_.strict_realtime) {
      running_.store(false, std::memory_order_release);
      start_gate_.store(true, std::memory_order_release);
      if (thread_.joinable()) {
        thread_.join();
      }
      return false;
    }

    start_gate_.store(true, std::memory_order_release);
    return true;
  }

  void stop() {
    running_.store(false, std::memory_order_release);
    start_gate_.store(true, std::memory_order_release);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  PeriodicThreadStats stats() const noexcept 
  {
    PeriodicThreadStats out;
    out.cycles = cycles_.load(std::memory_order_relaxed);
    out.overruns = overruns_.load(std::memory_order_relaxed);
    out.consecutive_overruns = consecutive_overruns_.load(std::memory_order_relaxed);
    out.max_exec_ns = max_exec_ns_.load(std::memory_order_relaxed);
    out.max_lateness_ns = max_lateness_ns_.load(std::memory_order_relaxed);
    out.last_exec_ns = last_exec_ns_.load(std::memory_order_relaxed);
    out.last_lateness_ns = last_lateness_ns_.load(std::memory_order_relaxed);
    out.sched_fifo_active = sched_fifo_active_.load(std::memory_order_relaxed);
    out.memory_locked = memory_locked_.load(std::memory_order_relaxed);
    out.cpu_affinity = active_cpu_.load(std::memory_order_relaxed);
    out.priority = active_priority_.load(std::memory_order_relaxed);
    return out;
  }

 private:
  static void updateMax(std::atomic<std::uint64_t>& dst, std::uint64_t value) {
    auto current = dst.load(std::memory_order_relaxed);
    while (value > current &&
           !dst.compare_exchange_weak(
               current, value,
               std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
  }

  void resetStats() noexcept {
    cycles_.store(0, std::memory_order_relaxed);
    overruns_.store(0, std::memory_order_relaxed);
    consecutive_overruns_.store(0, std::memory_order_relaxed);
    max_exec_ns_.store(0, std::memory_order_relaxed);
    max_lateness_ns_.store(0, std::memory_order_relaxed);
    last_exec_ns_.store(0, std::memory_order_relaxed);
    last_lateness_ns_.store(0, std::memory_order_relaxed);
    sched_fifo_active_.store(false, std::memory_order_relaxed);
    memory_locked_.store(false, std::memory_order_relaxed);
    active_cpu_.store(-1, std::memory_order_relaxed);
    active_priority_.store(0, std::memory_order_relaxed);
  }

  bool configureRealtime() {
#ifdef __linux__
    bool ok = true;

    if (options_.lock_memory) {
      if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
        memory_locked_.store(true, std::memory_order_relaxed);
      } else {
        ok = false;
        std::cerr << "[PeriodicThread] mlockall failed: "
                  << std::strerror(errno) << std::endl;
      }
    }

    if (options_.realtime) {
      sched_param param{};
      param.sched_priority = std::clamp(
          options_.priority,
          ::sched_get_priority_min(SCHED_FIFO),
          ::sched_get_priority_max(SCHED_FIFO));

      const int rc = ::pthread_setschedparam(
          thread_.native_handle(), SCHED_FIFO, &param);
      if (rc == 0) {
        sched_fifo_active_.store(true, std::memory_order_relaxed);
        active_priority_.store(param.sched_priority, std::memory_order_relaxed);
      } else {
        ok = false;
        std::cerr << "[PeriodicThread] SCHED_FIFO failed: "
                  << std::strerror(rc) << std::endl;
      }
    }

    if (options_.cpu_affinity >= 0) {
      if (options_.cpu_affinity >= CPU_SETSIZE) {
        ok = false;
        std::cerr << "[PeriodicThread] CPU affinity out of range: "
                  << options_.cpu_affinity << std::endl;
        return ok;
      }
      cpu_set_t set;
      CPU_ZERO(&set);
      CPU_SET(options_.cpu_affinity, &set);
      const int rc = ::pthread_setaffinity_np(
          thread_.native_handle(), sizeof(set), &set);
      if (rc == 0) {
        active_cpu_.store(options_.cpu_affinity, std::memory_order_relaxed);
      } else {
        ok = false;
        std::cerr << "[PeriodicThread] CPU affinity failed: "
                  << std::strerror(rc) << std::endl;
      }
    }

    return ok;
#else
    const bool requested = options_.realtime || options_.lock_memory ||
                           options_.cpu_affinity >= 0;
    if (requested) {
      std::cerr << "[PeriodicThread] Linux RT features are unavailable on this platform."
                << std::endl;
    }
    return !requested;
#endif
  }

#ifdef __linux__
  static std::uint64_t timespecToNs(const timespec& ts) noexcept {
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<std::uint64_t>(ts.tv_nsec);
  }

  static timespec nsToTimespec(std::uint64_t ns) noexcept {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(ns / 1000000000ULL);
    ts.tv_nsec = static_cast<long>(ns % 1000000000ULL);
    return ts;
  }
#endif

  void runLoop(const std::function<void()>& fn) {
#ifdef __linux__
    timespec now_ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &now_ts);
    std::uint64_t scheduled_ns = timespecToNs(now_ts);
    const std::uint64_t period_ns = static_cast<std::uint64_t>(period_.count());

    while (running_.load(std::memory_order_acquire)) {
      timespec begin_ts{};
      ::clock_gettime(CLOCK_MONOTONIC, &begin_ts);
      const std::uint64_t begin_ns = timespecToNs(begin_ts);
      const std::uint64_t lateness_ns =
          begin_ns > scheduled_ns ? begin_ns - scheduled_ns : 0ULL;

      fn();

      timespec end_ts{};
      ::clock_gettime(CLOCK_MONOTONIC, &end_ts);
      const std::uint64_t end_ns = timespecToNs(end_ts);
      const std::uint64_t exec_ns = end_ns - begin_ns;
      const std::uint64_t deadline_ns = scheduled_ns + period_ns;
      const bool overrun = end_ns > deadline_ns;

      cycles_.fetch_add(1, std::memory_order_relaxed);
      last_exec_ns_.store(exec_ns, std::memory_order_relaxed);
      last_lateness_ns_.store(lateness_ns, std::memory_order_relaxed);
      updateMax(max_exec_ns_, exec_ns);
      updateMax(max_lateness_ns_, lateness_ns);

      if (overrun) {
        overruns_.fetch_add(1, std::memory_order_relaxed);
        consecutive_overruns_.fetch_add(1, std::memory_order_relaxed);
      } else {
        consecutive_overruns_.store(0, std::memory_order_relaxed);
      }

      scheduled_ns = deadline_ns;
      if (!running_.load(std::memory_order_acquire)) {
        break;
      }

      if (end_ns < scheduled_ns) {
        const timespec deadline = nsToTimespec(scheduled_ns);
        int rc = 0;
        do {
          rc = ::clock_nanosleep(
              CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
        } while (rc == EINTR && running_.load(std::memory_order_acquire));
      } else {
        // If several periods were missed, skip old deadlines instead of
        // spinning to catch up.
        const std::uint64_t behind = end_ns - scheduled_ns;
        if (behind >= period_ns) {
          scheduled_ns += (behind / period_ns) * period_ns;
        }
      }
    }
#else
    auto scheduled = std::chrono::steady_clock::now();
    while (running_.load(std::memory_order_acquire)) {
      const auto begin = std::chrono::steady_clock::now();
      const auto lateness = begin > scheduled ? begin - scheduled : decltype(begin - scheduled)::zero();
      fn();
      const auto end = std::chrono::steady_clock::now();
      const auto deadline = scheduled + period_;
      const bool overrun = end > deadline;
      const auto exec = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
      const auto late = std::chrono::duration_cast<std::chrono::nanoseconds>(lateness).count();
      cycles_.fetch_add(1, std::memory_order_relaxed);
      last_exec_ns_.store(static_cast<std::uint64_t>(std::max<std::int64_t>(0, exec)), std::memory_order_relaxed);
      last_lateness_ns_.store(static_cast<std::uint64_t>(std::max<std::int64_t>(0, late)), std::memory_order_relaxed);
      if (overrun) {
        overruns_.fetch_add(1, std::memory_order_relaxed);
        consecutive_overruns_.fetch_add(1, std::memory_order_relaxed);
      } else {
        consecutive_overruns_.store(0, std::memory_order_relaxed);
      }
      scheduled = deadline;
      std::this_thread::sleep_until(scheduled);
    }
#endif
  }

  std::atomic_bool running_{false};
  std::atomic_bool start_gate_{false};
  std::thread thread_;
  std::chrono::nanoseconds period_{0};
  PeriodicThreadOptions options_{};

  std::atomic<std::uint64_t> cycles_{0};
  std::atomic<std::uint64_t> overruns_{0};
  std::atomic<std::uint64_t> consecutive_overruns_{0};
  std::atomic<std::uint64_t> max_exec_ns_{0};
  std::atomic<std::uint64_t> max_lateness_ns_{0};
  std::atomic<std::uint64_t> last_exec_ns_{0};
  std::atomic<std::uint64_t> last_lateness_ns_{0};
  std::atomic<bool> sched_fifo_active_{false};
  std::atomic<bool> memory_locked_{false};
  std::atomic<int> active_cpu_{-1};
  std::atomic<int> active_priority_{0};
};

}  // namespace noetix

#endif  // NOETIX_COMMON_PERIODIC_THREAD_HPP_
