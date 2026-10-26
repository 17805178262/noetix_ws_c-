#include "usb_imu_reader.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

namespace noetix {
namespace {
constexpr std::uint8_t kHeader1 = 0x59U;
constexpr std::uint8_t kHeader2 = 0x53U;
constexpr std::size_t kProtocolMinBytes = 7U;
constexpr std::size_t kPayloadOffset = 5U;
constexpr std::uint8_t kAccelId = 0x10U;
constexpr std::uint8_t kGyroId = 0x20U;
constexpr std::uint8_t kQuaternionId = 0x41U;
constexpr std::uint8_t kVectorBytes = 12U;
constexpr std::uint8_t kQuaternionBytes = 16U;
constexpr float kProtocolScale = 1.0e-6F;
// Keep the conversion used by the original robot IMU driver.
constexpr float kDegPerRadOriginal = 57.325F;
constexpr std::uint64_t kSampleTimeoutNs = 500000000ULL;
constexpr int kSerialThreadPriority = 94;
constexpr int kPollTimeoutMs = 20;
}  // namespace

UsbImuReader::UsbImuReader()
{
  buffer_.initialize(ImuState{});
}

UsbImuReader::~UsbImuReader()
{
  stop();
}

std::uint64_t UsbImuReader::steadyNowNs() noexcept
{
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool UsbImuReader::start(const UsbImuConfig& config)
{
  stop();
  if (config.port.empty()) {
    std::cerr << "[UsbImuReader] serial port is empty." << std::endl;
    return false;
  }

  config_ = config;
  receive_size_ = 0U;
  decoded_ = DecodedData{};
  last_sample_ns_.store(0U, std::memory_order_release);
  buffer_.write(ImuState{});

  if (!openPort()) {
    return false;
  }

  running_.store(true, std::memory_order_release);
  thread_ = std::thread(&UsbImuReader::run, this);

  sched_param sched{};
  sched.sched_priority = kSerialThreadPriority;
  if (::pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    std::cerr << "[UsbImuReader] failed to set SCHED_FIFO priority "
              << kSerialThreadPriority
              << "; continuing with normal scheduling." << std::endl;
  }
  return true;
}

void UsbImuReader::stop()
{
  running_.store(false, std::memory_order_release);
  if (thread_.joinable()) {
    thread_.join();
  }
  closePort();
  last_sample_ns_.store(0U, std::memory_order_release);
}

bool UsbImuReader::running() const noexcept
{
  return running_.load(std::memory_order_acquire);
}

bool UsbImuReader::openPort()
{
#ifndef B921600
  std::cerr << "[UsbImuReader] B921600 is not supported by this platform." << std::endl;
  return false;
#else
  fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    std::cerr << "[UsbImuReader] open " << config_.port
              << " failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[UsbImuReader] tcgetattr failed: " << std::strerror(errno) << std::endl;
    closePort();
    return false;
  }

  ::cfmakeraw(&tty);
  ::cfsetispeed(&tty, B921600);
  ::cfsetospeed(&tty, B921600);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSAFLUSH, &tty) != 0) {
    std::cerr << "[UsbImuReader] tcsetattr failed: " << std::strerror(errno) << std::endl;
    closePort();
    return false;
  }

  // Flush only once when opening. Flushing every read cycle can discard valid
  // bytes belonging to the next frame.
  ::tcflush(fd_, TCIFLUSH);
  return true;
#endif
}

void UsbImuReader::closePort()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::uint16_t UsbImuReader::checksum(const std::uint8_t* data,
                                     std::size_t len) noexcept
{
  std::uint8_t check_a = 0U;
  std::uint8_t check_b = 0U;
  for (std::size_t i = 0; i < len; ++i) {
    check_a = static_cast<std::uint8_t>(check_a + data[i]);
    check_b = static_cast<std::uint8_t>(check_b + check_a);
  }
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(check_b) << 8U) | check_a);
}

std::int32_t UsbImuReader::readLeI32(const std::uint8_t* data) noexcept
{
  const std::uint32_t value =
      static_cast<std::uint32_t>(data[0]) |
      (static_cast<std::uint32_t>(data[1]) << 8U) |
      (static_cast<std::uint32_t>(data[2]) << 16U) |
      (static_cast<std::uint32_t>(data[3]) << 24U);
  return static_cast<std::int32_t>(value);
}

bool UsbImuReader::parseFrame(const std::uint8_t* frame, std::size_t frame_size)
{
  if (frame == nullptr || frame_size < kProtocolMinBytes ||
      frame[0] != kHeader1 || frame[1] != kHeader2) {
    return false;
  }

  const std::size_t payload_len = frame[4];
  if (frame_size != payload_len + kProtocolMinBytes) {
    return false;
  }

  const std::size_t checksum_input_len = payload_len + 3U;
  const std::uint16_t expected = checksum(frame + 2U, checksum_input_len);
  const std::size_t checksum_pos = kPayloadOffset + payload_len;
  const std::uint16_t received =
      static_cast<std::uint16_t>(frame[checksum_pos]) |
      (static_cast<std::uint16_t>(frame[checksum_pos + 1U]) << 8U);
  if (expected != received) {
    return false;
  }

  std::size_t pos = kPayloadOffset;
  const std::size_t payload_end = kPayloadOffset + payload_len;
  bool parsed_any = false;

  while (pos + 2U <= payload_end) {
    const std::uint8_t id = frame[pos];
    const std::uint8_t data_len = frame[pos + 1U];
    pos += 2U;
    if (pos + data_len > payload_end) {
      return false;
    }

    const std::uint8_t* data = frame + pos;
    if (id == kAccelId && data_len == kVectorBytes) {
      for (std::size_t i = 0; i < 3U; ++i) {
        decoded_.accel[i] = static_cast<float>(readLeI32(data + i * 4U)) * kProtocolScale;
      }
      decoded_.have_accel = true;
      parsed_any = true;
    } else if (id == kGyroId && data_len == kVectorBytes) {
      for (std::size_t i = 0; i < 3U; ++i) {
        decoded_.gyro_deg[i] = static_cast<float>(readLeI32(data + i * 4U)) * kProtocolScale;
      }
      decoded_.have_gyro = true;
      parsed_any = true;
    } else if (id == kQuaternionId && data_len == kQuaternionBytes) {
      for (std::size_t i = 0; i < 4U; ++i) {
        decoded_.quaternion[i] =
            static_cast<float>(readLeI32(data + i * 4U)) * kProtocolScale;
      }
      decoded_.have_quaternion = true;
      parsed_any = true;
    }

    pos += data_len;
  }

  if (pos != payload_end) {
    return false;
  }

  if (parsed_any && decoded_.have_accel && decoded_.have_gyro &&
      decoded_.have_quaternion) {
    publishDecodedSample();
  }
  return parsed_any;
}

void UsbImuReader::publishDecodedSample()
{
  ImuState imu{};
  imu.accelerometer = decoded_.accel;
  for (std::size_t i = 0; i < 3U; ++i) {
    imu.gyroscope[i] = decoded_.gyro_deg[i] / kDegPerRadOriginal;
  }
  imu.quaternion = decoded_.quaternion;

  bool finite = true;
  for (const float value : imu.accelerometer) finite = finite && std::isfinite(value);
  for (const float value : imu.gyroscope) finite = finite && std::isfinite(value);
  for (const float value : imu.quaternion) finite = finite && std::isfinite(value);

  const double qnorm2 =
      static_cast<double>(imu.quaternion[0]) * imu.quaternion[0] +
      static_cast<double>(imu.quaternion[1]) * imu.quaternion[1] +
      static_cast<double>(imu.quaternion[2]) * imu.quaternion[2] +
      static_cast<double>(imu.quaternion[3]) * imu.quaternion[3];
  imu.valid = finite && qnorm2 > 1.0e-8;
  if (!imu.valid) {
    return;
  }

  buffer_.write(imu);
  last_sample_ns_.store(steadyNowNs(), std::memory_order_release);
}

void UsbImuReader::processReceiveBuffer()
{
  std::size_t pos = 0U;
  while (receive_size_ - pos >= kProtocolMinBytes) {
    if (receive_buffer_[pos] != kHeader1 || receive_buffer_[pos + 1U] != kHeader2) {
      ++pos;
      continue;
    }

    const std::size_t frame_size =
        static_cast<std::size_t>(receive_buffer_[pos + 4U]) + kProtocolMinBytes;
    if (receive_size_ - pos < frame_size) {
      break;
    }

    // CRC failure or a malformed payload only drops this complete candidate
    // frame. Header resynchronization happens on the next iteration.
    if (parseFrame(receive_buffer_.data() + pos, frame_size)) {
      pos += frame_size;
    } else {
      ++pos;
    }
  }

  if (pos > 0U) {
    const std::size_t remaining = receive_size_ - pos;
    if (remaining > 0U) {
      std::memmove(receive_buffer_.data(), receive_buffer_.data() + pos, remaining);
    }
    receive_size_ = remaining;
  }
}

void UsbImuReader::run()
{
  std::array<std::uint8_t, 512> read_buffer{};

  while (running_.load(std::memory_order_acquire)) {
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int rc = ::poll(&pfd, 1, kPollTimeoutMs);
    if (rc < 0) {
      if (errno == EINTR) continue;
      std::cerr << "[UsbImuReader] poll failed: " << std::strerror(errno) << std::endl;
      break;
    }
    if (rc == 0) continue;
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      std::cerr << "[UsbImuReader] serial device disconnected or failed." << std::endl;
      break;
    }
    if ((pfd.revents & POLLIN) == 0) continue;

    const ssize_t count = ::read(fd_, read_buffer.data(), read_buffer.size());
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      std::cerr << "[UsbImuReader] read failed: " << std::strerror(errno) << std::endl;
      break;
    }
    if (count == 0) continue;

    const std::size_t incoming = static_cast<std::size_t>(count);
    if (incoming > receive_buffer_.size() - receive_size_) {
      // Try to parse/compact first. If the stream is still too damaged to fit,
      // reset the parser buffer instead of writing past the fixed array.
      processReceiveBuffer();
      if (incoming > receive_buffer_.size() - receive_size_) {
        receive_size_ = 0U;
      }
    }

    std::memcpy(receive_buffer_.data() + receive_size_, read_buffer.data(), incoming);
    receive_size_ += incoming;
    processReceiveBuffer();
  }

  running_.store(false, std::memory_order_release);
}

bool UsbImuReader::read(ImuState& imu) const
{
  const std::uint64_t sample_ns = last_sample_ns_.load(std::memory_order_acquire);
  if (sample_ns == 0U || steadyNowNs() - sample_ns > kSampleTimeoutNs ||
      !buffer_.read(imu) || !imu.valid) {
    imu.valid = false;
    return false;
  }
  return true;
}

}  // namespace noetix
