#ifndef NOETIX_BUMI_USB_IMU_READER_HPP_
#define NOETIX_BUMI_USB_IMU_READER_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

#include "noetix/common/realtime_buffer.hpp"
#include "noetix/robot/imu_state.hpp"

namespace noetix {

struct UsbImuConfig {
  std::string port{"/dev/ttyUSB0"};
};

/**
 * USB IMU reader for the original Bumi/legged YeSense binary protocol.
 *
 * Configuration intentionally stays minimal: the caller only decides whether
 * USB is enabled and which serial port to open. The wire format and 921600 baud
 * rate are fixed by the hardware protocol.
 *
 * Serial I/O and frame parsing run on a dedicated thread. The EtherCAT/control
 * thread only reads the latest snapshot from RealtimeBuffer.
 */
class UsbImuReader {
 public:
  UsbImuReader();
  ~UsbImuReader();

  UsbImuReader(const UsbImuReader&) = delete;
  UsbImuReader& operator=(const UsbImuReader&) = delete;

  bool start(const UsbImuConfig& config);
  void stop();
  bool read(ImuState& imu) const;
  bool running() const noexcept;

 private:
  struct DecodedData {
    std::array<float, 3> accel{};
    std::array<float, 3> gyro_deg{};
    std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    bool have_accel{false};
    bool have_gyro{false};
    bool have_quaternion{false};
  };

  void run();
  bool openPort();
  void closePort();
  void processReceiveBuffer();
  bool parseFrame(const std::uint8_t* frame, std::size_t frame_size);
  void publishDecodedSample();

  static std::uint16_t checksum(const std::uint8_t* data, std::size_t len) noexcept;
  static std::int32_t readLeI32(const std::uint8_t* data) noexcept;
  static std::uint64_t steadyNowNs() noexcept;

  UsbImuConfig config_;
  int fd_{-1};
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> last_sample_ns_{0};
  RealtimeBuffer<ImuState> buffer_;

  std::array<std::uint8_t, 2048> receive_buffer_{};
  std::size_t receive_size_{0};
  DecodedData decoded_{};
};

}  // namespace noetix

#endif  // NOETIX_BUMI_USB_IMU_READER_HPP_
