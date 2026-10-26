#pragma once
/**
 * xsens_reader.hpp — Xsens MVN UDP Network Streamer reader
 *
 * Implements Xsens MVN real-time network streaming protocol (MV0305P.N)
 * Type 02: Quaternion pose data, Z-Up right-handed, positions in cm.
 *
 * No external SDK required — pure UDP socket + Eigen.
 *
 * Drop-in replacement for OptiTrackReader:
 *   same BaseReader interface, same FrameQueue output.
 */

#include "base_reader.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <string>
#include <thread>
#include <unordered_map>

namespace gmr {

class XsensReader : public BaseReader {
public:
    struct Config {
        int    port    = 9763;   // Xsens MVN default UDP port
        bool   verbose = true;
    };

    XsensReader(FrameQueue& queue, Config cfg);
    ~XsensReader() override;

    void connect()    override;
    void disconnect() override;

    std::string name()        const override { return "Xsens"; }
    bool        isConnected() const override { return connected_; }

    // Recapture yaw normalization on next frame
    void resetYawNormalization();

private:
    void readerLoop();
    bool parsePacket(const uint8_t* buf, int len, gmr::BodyMap& out);
    void applyYawNorm(gmr::BodyMap& data);
    bool validateFrame(const gmr::BodyMap& data) const;

    Config            cfg_;
    int               sock_fd_    {-1};
    std::atomic<bool> connected_  {false};
    std::atomic<bool> stop_flag_  {false};
    std::thread       reader_thread_;

    // Yaw normalization state
    bool               yaw_captured_ {false};
    Eigen::Quaterniond yaw_inv_;

    // Last seen sample counter (detect new frames)
    uint32_t last_sample_counter_ {0xFFFFFFFF};

    // ── Protocol constants ────────────────────────────────────────────────
    // Segment index → GMR canonical name (Table 1, type 02 quaternion)
    // Index = segment_id - 1  (protocol sends 1-based IDs)
    static const std::unordered_map<int, std::string> kSegmentNames;
    static const std::vector<std::string>             kRequiredBodies;

    static constexpr int    HEADER_SIZE      = 24;
    static constexpr int    SEGMENT_SIZE     = 32;  // type 02: id(4)+xyz(12)+quat(16)
    static constexpr float  CM_TO_M          = 1.0f;
    static constexpr char   ID_STRING[6]     = {'M','X','T','P','0','2'};
};

} // namespace gmr