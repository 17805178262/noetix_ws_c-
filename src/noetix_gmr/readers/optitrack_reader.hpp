#pragma once
/**
 * optitrack_reader.hpp — NatNet SDK 4.x reader
 *
 * Connects to Motive, receives skeleton frames via callback,
 * maps FBX bone names to canonical names, pushes into FrameQueue.
 */

#include "base_reader.hpp"
#include <NatNetClient.h>
#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <map>
#include <string>
#include <memory>
#include <atomic>

namespace gmr {

class OptiTrackReader : public BaseReader {
public:
    struct Config {
        std::string server_ip;
        std::string client_ip;
        // FBX bone name → canonical name used in IK config JSON
        // Defaults to the standard FBX→G1 mapping if left empty
        std::map<std::string, std::string> bone_name_map;
    };

    OptiTrackReader(FrameQueue& queue, Config cfg);
    ~OptiTrackReader() override;

    void connect()    override;
    void disconnect() override;
    std::string name() const override { return "OptiTrack"; }
    bool isConnected() const override { return connected_; }

private:
    static void NATNET_CALLCONV dataCallback(
        sFrameOfMocapData* data, void* user);

    void buildBoneNameMap();

    Config                              cfg_;
    std::unique_ptr<NatNetClient>       client_;
    std::map<int, std::string>          bone_id_to_name_;
    std::atomic<bool>                   connected_{false};

    // Default FBX bone index → canonical name table (1-based, matches Motive stream)
    static const std::vector<std::string> kDefaultBoneNames;

    // Default FBX → canonical mapping
    static const std::map<std::string, std::string> kDefaultBoneNameMap;
};

} // namespace gmr
