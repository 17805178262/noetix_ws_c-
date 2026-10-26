#pragma once
/**
 * fzmotion_reader.hpp — Lusterinc LuMoSDK reader for FZMotion mocap system
 *
 * Subscribes to FZMotion mocap data via the official LuMo SDK. Maps FZMotion
 * bone names to the canonical FBX names already used by the OptiTrack pipeline
 * so downstream GMR / IK configs can stay unchanged.
 */

#include "base_reader.hpp"
#include "LuMoSDKBase.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

namespace gmr {

class FzmotionReader : public BaseReader {
public:
    struct Config {
        // FZMotion host IP. The LuMo SDK uses its configured port internally.
        std::string server_ip;

        // Optional: only accept skeletons whose BodyName matches this string.
        // Empty = accept any tracked skeleton.
        std::string subject_filter;

        // FZMotion positions are in millimeters; the GMR pipeline expects meters.
        double position_scale = 1e-3;

        // FZMotion bone name -> canonical FBX name. Empty uses kDefaultFzToCanonical.
        std::map<std::string, std::string> bone_name_map;

        bool verbose = true;
    };

    FzmotionReader(FrameQueue& queue, Config cfg);
    ~FzmotionReader() override;

    void connect() override;
    void disconnect() override;

    std::string name() const override { return "Fzmotion"; }
    bool isConnected() const override { return connected_; }

    // Default FZMotion -> canonical FBX bone mapping.
    static const std::map<std::string, std::string> kDefaultFzToCanonical;

private:
    void readerLoop();

    Config cfg_;
    std::shared_ptr<lusternet::CReceiveBase> sdk_;
    std::thread reader_thread_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_flag_{false};
    uint64_t frame_counter_{0};
};

} // namespace gmr
