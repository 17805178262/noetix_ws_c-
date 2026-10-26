#include "xsens_reader.hpp"
#include "gmr/frame_queue.hpp"

#include <Eigen/Geometry>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <chrono>

// POSIX socket headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace gmr {

// ── Static tables ─────────────────────────────────────────────────────────────
// Xsens protocol Table 1 (type 02 quaternion), segment ID is 1-based.
// Keys here are segment_id (1-based) matching what comes over the wire.
const std::unordered_map<int, std::string> XsensReader::kSegmentNames = {
    { 1, "Pelvis"},
    { 2, "Spine"},           // L5
    { 3, "Spine1"},          // L3
    { 4, "Spine2"},          // T12
    { 5, "Chest"},           // T8
    { 6, "Neck"},
    { 7, "Head"},
    { 8, "Right_Shoulder"},
    { 9, "Right_UpperArm"},
    {10, "Right_Forearm"},
    {11, "Right_Hand"},
    {12, "Left_Shoulder"},
    {13, "Left_UpperArm"},
    {14, "Left_Forearm"},
    {15, "Left_Hand"},
    {16, "Right_UpperLeg"},
    {17, "Right_LowerLeg"},
    {18, "Right_Foot"},
    {19, "Right_Toe"},
    {20, "Left_UpperLeg"},
    {21, "Left_LowerLeg"},
    {22, "Left_Foot"},
    {23, "Left_Toe"},
};

const std::vector<std::string> XsensReader::kRequiredBodies = {
    "Pelvis", "Head",
    "Left_Hand",  "Right_Hand",
    "Left_Foot",  "Right_Foot",
};

// ── Constructor / Destructor ──────────────────────────────────────────────────
XsensReader::XsensReader(FrameQueue& queue, Config cfg)
    : BaseReader(queue), cfg_(std::move(cfg)) {}

XsensReader::~XsensReader() {
    disconnect();
}

// ── connect() ─────────────────────────────────────────────────────────────────
void XsensReader::connect() {
    // Create UDP socket
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0)
        throw std::runtime_error("[Xsens] Failed to create socket");

    // Allow reuse in case of quick restart
    int opt = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 加大接收缓冲区防止丢包
    int rcvbuf = 4 * 1024 * 1024;  // 4MB
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // Set receive timeout so readerLoop can check stop_flag_ periodically
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 100000;  // 100ms timeout
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // Bind to the Xsens MVN streaming port
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(cfg_.port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error("[Xsens] Failed to bind UDP socket on port "
                                 + std::to_string(cfg_.port));
    }

    connected_  = true;
    stop_flag_  = false;

    reader_thread_ = std::thread(&XsensReader::readerLoop, this);

    if (cfg_.verbose)
        std::printf("[Xsens] Listening on UDP port %d\n", cfg_.port);
}

// ── disconnect() ─────────────────────────────────────────────────────────────
void XsensReader::disconnect() {
    stop_flag_ = true;

    if (reader_thread_.joinable())
        reader_thread_.join();

    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }

    connected_ = false;

    if (cfg_.verbose)
        std::cout << "[Xsens] Disconnected.\n";
}

// ── readerLoop() ─────────────────────────────────────────────────────────────
// Sits in background, receives UDP packets, parses, pushes to queue.
void XsensReader::readerLoop() {
    // Max UDP packet: 24-byte header + 23 segments * 32 bytes + fingers
    static constexpr int BUF_SIZE = 4096;
    uint8_t buf[BUF_SIZE];

    while (!stop_flag_) {
        int n = recv(sock_fd_, buf, BUF_SIZE, 0);

        if (n < 0) {
            // EAGAIN/EWOULDBLOCK = timeout, just loop and check stop_flag_
            continue;
        }

        if (n < HEADER_SIZE) continue;  // too short, skip

        // ── Validate ID string (first 6 bytes = "MXTP02") ────────────────
        // We only want type 02 (quaternion pose data)
        if (std::memcmp(buf, ID_STRING, 6) != 0) continue;

        // ── Read sample counter (bytes 6-9, big-endian uint32) ────────────
        uint32_t sample_counter;
        std::memcpy(&sample_counter, buf + 6, 4);
        sample_counter = ntohl(sample_counter);

        // Skip if same frame as last time
        if (sample_counter == last_sample_counter_) continue;
        last_sample_counter_ = sample_counter;

        // ── Parse segments into BodyMap ───────────────────────────────────
        gmr::RawFrame frame;
        if (!parsePacket(buf, n, frame.body_data)) continue;

        applyYawNorm(frame.body_data);

        // ── Timestamp and push ────────────────────────────────────────────
        frame.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        queue_.push(std::move(frame));
    }
}

// ── parsePacket() ─────────────────────────────────────────────────────────────
// Parses type-02 quaternion datagram into gmr::BodyMap.
// Protocol spec section 2.5.3:
//   Per segment: segment_id(4) + x(4) + y(4) + z(4) + q1(4) + q2(4) + q3(4) + q4(4)
//   Position unit: cm  → convert to meters
//   Quaternion:  [re, i, j, k] = [w, x, y, z], scalar-first
//   Coordinate system: Z-Up right-handed
//   All values: big-endian float32
bool XsensReader::parsePacket(const uint8_t* buf, int len, gmr::BodyMap& out) {
    out.clear();

    // num_items is at byte 11 (1 byte)
    uint8_t num_items = buf[11];

    // Payload starts at byte HEADER_SIZE
    const uint8_t* ptr = buf + HEADER_SIZE;
    const uint8_t* end = buf + len;

    for (int i = 0; i < num_items; ++i) {
        if (ptr + SEGMENT_SIZE > end) break;  // incomplete packet

        // ── Segment ID (4 bytes, big-endian int32) ────────────────────────
        int32_t seg_id_raw;
        std::memcpy(&seg_id_raw, ptr, 4);
        int seg_id = static_cast<int>(ntohl(static_cast<uint32_t>(seg_id_raw)));
        ptr += 4;

        // ── Position (3 × float32 big-endian, unit: cm) ───────────────────
        float x_raw, y_raw, z_raw;
        uint32_t tmp;

        std::memcpy(&tmp, ptr + 0, 4); tmp = ntohl(tmp);
        std::memcpy(&x_raw, &tmp, 4);

        std::memcpy(&tmp, ptr + 4, 4); tmp = ntohl(tmp);
        std::memcpy(&y_raw, &tmp, 4);

        std::memcpy(&tmp, ptr + 8, 4); tmp = ntohl(tmp);
        std::memcpy(&z_raw, &tmp, 4);
        ptr += 12;

        // cm → meters
        Eigen::Vector3d pos(
            static_cast<double>(x_raw) * CM_TO_M,
            static_cast<double>(y_raw) * CM_TO_M,
            static_cast<double>(z_raw) * CM_TO_M
        );

        // ── Quaternion (4 × float32 big-endian, [w, x, y, z]) ─────────────
        float qw_raw, qx_raw, qy_raw, qz_raw;

        std::memcpy(&tmp, ptr + 0,  4); tmp = ntohl(tmp); std::memcpy(&qw_raw, &tmp, 4);
        std::memcpy(&tmp, ptr + 4,  4); tmp = ntohl(tmp); std::memcpy(&qx_raw, &tmp, 4);
        std::memcpy(&tmp, ptr + 8,  4); tmp = ntohl(tmp); std::memcpy(&qy_raw, &tmp, 4);
        std::memcpy(&tmp, ptr + 12, 4); tmp = ntohl(tmp); std::memcpy(&qz_raw, &tmp, 4);
        ptr += 16;

        // ── Map segment ID → GMR body name ───────────────────────────────
        auto it = kSegmentNames.find(seg_id);
        if (it == kSegmentNames.end()) continue;  // prop or finger, skip
        const std::string& body_name = it->second;

        // ── Store in BodyMap ──────────────────────────────────────────────
        gmr::BodyData bd;
        bd.position = pos;
        // rot_wxyz matches Xsens quaternion order [re, i, j, k] = [w, x, y, z]
        bd.rot_wxyz = Eigen::Vector4d(qw_raw, qx_raw, qy_raw, qz_raw);

        out[body_name] = bd;
    }

    return validateFrame(out);
}

// ── validateFrame() ───────────────────────────────────────────────────────────
bool XsensReader::validateFrame(const gmr::BodyMap& data) const {
    for (const auto& req : kRequiredBodies)
        if (data.find(req) == data.end()) return false;
    return true;
}

// ── applyYawNorm() ────────────────────────────────────────────────────────────
// Mirrors Python _apply_yaw_normalization() exactly:
//   First frame  → extract pelvis yaw, store inverse
//   Every frame  → rotate all positions and orientations by inverse yaw
void XsensReader::applyYawNorm(gmr::BodyMap& data) {
    auto it = data.find("Pelvis");
    if (it == data.end()) return;

    if (!yaw_captured_) {
        const Eigen::Vector4d& q = it->second.rot_wxyz;
        Eigen::Quaterniond pelvis_rot(q[0], q[1], q[2], q[3]);
        pelvis_rot.normalize();

        // Extract yaw (Z-axis rotation) using ZYX Euler convention
        // eulerAngles(2,1,0) → [yaw, pitch, roll]
        Eigen::Matrix3d R = pelvis_rot.toRotationMatrix();
        double yaw = std::atan2(R(1,0), R(0,0));  // 正确的 Z 轴 yaw 提取

        yaw_inv_ = Eigen::Quaterniond(
            Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));

        yaw_captured_ = true;

        if (cfg_.verbose)
            std::printf("[Xsens] Initial pelvis yaw: %.1f° → normalized to 0°\n",
                        yaw * 180.0 / M_PI);
    }

    for (auto& [name, bd] : data) {
        // Rotate position around world Z
        bd.position = yaw_inv_ * bd.position;

        // Pre-multiply orientation: new_rot = yaw_inv * body_rot
        Eigen::Quaterniond body_rot(
            bd.rot_wxyz[0], bd.rot_wxyz[1],
            bd.rot_wxyz[2], bd.rot_wxyz[3]);
        Eigen::Quaterniond new_rot = (yaw_inv_ * body_rot).normalized();
        bd.rot_wxyz = Eigen::Vector4d(
            new_rot.w(), new_rot.x(), new_rot.y(), new_rot.z());
    }
}

// ── resetYawNormalization() ───────────────────────────────────────────────────
void XsensReader::resetYawNormalization() {
    yaw_captured_ = false;
    if (cfg_.verbose)
        std::cout << "[Xsens] Yaw normalization reset.\n";
}

} // namespace gmr