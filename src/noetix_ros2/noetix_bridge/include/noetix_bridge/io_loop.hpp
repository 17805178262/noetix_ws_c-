#ifndef NOETIX_BRIDGE__IO_LOOP_HPP_
#define NOETIX_BRIDGE__IO_LOOP_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "noetix/common/periodic_thread.hpp"
#include "noetix/common/realtime_buffer.hpp"
#include "noetix/robot/robot_interface.hpp"

namespace noetix_bridge {

/**
 * Thin periodic I/O pump.
 *
 * Responsibilities are intentionally limited to three things:
 *   1) periodically read the backend;
 *   2) while enabled, write the latest full LowCmd exactly as received;
 *   3) write an all-zero MIT packet when disabled or before the first command.
 *
 * It does not implement control modes, command freshness, safety shaping,
 * pose ownership, arbitration, fallback holds, or policy logic.
 */
class IoLoop {
 public:
  IoLoop(std::shared_ptr<noetix::RobotInterface> robot,
         std::vector<std::string> actuator_names);
         
  ~IoLoop();

  bool start(double frequency,
             const noetix::PeriodicThreadOptions& options = {});
  void stop();

  bool setCommand(const noetix::LowCmd& command);
  void clearCommand() noexcept;
  bool hasCommand() const noexcept;

  bool enableDrive();
  bool disableDrive();
  bool driveEnabled() const noexcept;

  bool getLatestState(noetix::LowState& state) const;
  noetix::PeriodicThreadStats stats() const noexcept;
  std::uint64_t readErrors() const noexcept;
  std::uint64_t writeErrors() const noexcept;

 private:
  void cycle();
  void initializeCommand(noetix::LowCmd& cmd) const;
  void zeroCommand(noetix::LowCmd& cmd) const noexcept;
  bool finiteCommand(const noetix::LowCmd& cmd) const noexcept;

  std::shared_ptr<noetix::RobotInterface> robot_;
  std::vector<std::string> actuator_names_;
  std::unordered_map<std::string, std::size_t> actuator_index_;
  std::size_t dof_{0};

  // RobotInterface itself is deliberately not thread-safe.  The periodic I/O
  // cycle and the infrequent enable/disable calls share this one mutex so the
  // vendor backend is never entered concurrently.
  mutable std::mutex robot_mutex_;

  noetix::LowState state_;
  noetix::LowCmd zero_command_;
  noetix::LowCmd command_snapshot_;
  noetix::RealtimeBuffer<noetix::LowState> state_buffer_;
  noetix::RealtimeBuffer<noetix::LowCmd> command_buffer_;

  std::atomic<bool> running_{false};
  std::atomic<bool> drive_enabled_{false};
  std::atomic<bool> command_valid_{false};
  std::atomic<std::uint64_t> read_errors_{0};
  std::atomic<std::uint64_t> write_errors_{0};
  std::uint64_t state_sequence_{0};

  noetix::PeriodicThread io_thread_;
};

}  // namespace noetix_bridge

#endif  // NOETIX_BRIDGE__IO_LOOP_HPP_
