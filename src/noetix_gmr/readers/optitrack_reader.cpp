/**
 * optitrack_reader.cpp
 */

#include "optitrack_reader.hpp"
#include <iostream>
#include <stdexcept>

namespace gmr {

// ── Static bone tables ────────────────────────────────────────────────────────

const std::vector<std::string> OptiTrackReader::kDefaultBoneNames = {
    "",              // 0 unused
    "Hips",          // 1
    "Spine",         // 2
    "Spine1",        // 3
    "Neck",          // 4
    "Head",          // 5
    "LeftShoulder",  // 6
    "LeftArm",       // 7
    "LeftForeArm",   // 8
    "LeftHand",      // 9
    "RightShoulder", // 10
    "RightArm",      // 11
    "RightForeArm",  // 12
    "RightHand",     // 13
    "LeftUpLeg",     // 14
    "LeftLeg",       // 15
    "LeftFoot",      // 16
    "LeftToeBase",   // 17
    "RightUpLeg",    // 18
    "RightLeg",      // 19
    "RightFoot",     // 20
    "RightToeBase",  // 21
};

// Default: FBX name == canonical name (identity mapping)
// Override in Config::bone_name_map if your IK config uses different names
const std::map<std::string, std::string> OptiTrackReader::kDefaultBoneNameMap = {
    {"Hips",           "Hips"},
    {"Spine",          "Spine"},
    {"Spine1",         "Spine1"},
    {"Spine2",         "Spine2"},
    {"Spine3",         "Spine3"},
    {"Neck",           "Neck"},
    {"Head",           "Head"},
    {"LeftShoulder",   "LeftShoulder"},
    {"LeftArm",        "LeftArm"},
    {"LeftForeArm",    "LeftForeArm"},
    {"LeftHand",       "LeftHand"},
    {"RightShoulder",  "RightShoulder"},
    {"RightArm",       "RightArm"},
    {"RightForeArm",   "RightForeArm"},
    {"RightHand",      "RightHand"},
    {"LeftUpLeg",      "LeftUpLeg"},
    {"LeftLeg",        "LeftLeg"},
    {"LeftFoot",       "LeftFoot"},
    {"LeftToeBase",    "LeftToeBase"},
    {"RightUpLeg",     "RightUpLeg"},
    {"RightLeg",       "RightLeg"},
    {"RightFoot",      "RightFoot"},
    {"RightToeBase",   "RightToeBase"},
};

// ── Constructor / Destructor ──────────────────────────────────────────────────

OptiTrackReader::OptiTrackReader(FrameQueue& queue, Config cfg)
    : BaseReader(queue), cfg_(std::move(cfg))
{
    // Fill default bone name map if caller didn't supply one
    if (cfg_.bone_name_map.empty())
        cfg_.bone_name_map = kDefaultBoneNameMap;
}

OptiTrackReader::~OptiTrackReader() {
    disconnect();
}

// ── connect / disconnect ──────────────────────────────────────────────────────

void OptiTrackReader::connect() {
    client_ = std::make_unique<NatNetClient>();

    sNatNetClientConnectParams p;
    p.connectionType = ConnectionType_Unicast;
    p.serverAddress  = cfg_.server_ip.c_str();
    p.localAddress   = cfg_.client_ip.c_str();

    if (client_->Connect(p) != ErrorCode_OK)
        throw std::runtime_error("[OptiTrackReader] Connect failed to " + cfg_.server_ip);

    buildBoneNameMap();

    // Pass 'this' as user pointer so the static callback can reach the queue
    client_->SetFrameReceivedCallback(dataCallback, this);
    connected_ = true;

    std::cout << "[OptiTrackReader] Connected. server=" << cfg_.server_ip
              << " client=" << cfg_.client_ip << "\n";
}

void OptiTrackReader::disconnect() {
    if (client_ && connected_) {
        client_->Disconnect();
        connected_ = false;
        std::cout << "[OptiTrackReader] Disconnected.\n";
    }
}

// ── buildBoneNameMap ──────────────────────────────────────────────────────────

void OptiTrackReader::buildBoneNameMap() {
    sDataDescriptions* desc = nullptr;
    if (client_->GetDataDescriptionList(&desc) != ErrorCode_OK || !desc) {
        std::cerr << "[OptiTrackReader] Could not get data descriptions\n";
        return;
    }

    for (int i = 0; i < desc->nDataDescriptions; ++i) {
        sDataDescription& d = desc->arrDataDescriptions[i];
        if (d.type != Descriptor_Skeleton) continue;
        sSkeletonDescription* sd = d.Data.SkeletonDescription;
        for (int b = 0; b < sd->nRigidBodies; ++b) {
            sRigidBodyDescription& rbd = sd->RigidBodies[b];
            bone_id_to_name_[rbd.ID] = rbd.szName;
        }
    }
    NatNet_FreeDescriptions(desc);

    std::cout << "[OptiTrackReader] Bones from Motive ("
              << bone_id_to_name_.size() << " total):\n";
    for (auto& [id, name] : bone_id_to_name_) {
        bool mapped = cfg_.bone_name_map.count(name) > 0;
        std::cout << "  ID=" << id << "  \"" << name << "\""
                  << (mapped ? "  → mapped" : "  ← NOT MAPPED") << "\n";
    }
}

// ── NatNet data callback (static) ─────────────────────────────────────────────

void NATNET_CALLCONV OptiTrackReader::dataCallback(
    sFrameOfMocapData* data, void* user)
{
    if (!data || !user) return;
    auto* self = static_cast<OptiTrackReader*>(user);

    RawFrame raw;
    raw.frame_number = data->iFrame;
    raw.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    for (int s = 0; s < data->nSkeletons; ++s) {
        sSkeletonData& skel = data->Skeletons[s];
        for (int b = 0; b < skel.nRigidBodies; ++b) {
            sRigidBodyData& rb = skel.RigidBodyData[b];

            // Lower 16 bits = bone index (1-based)
            int bone_idx = rb.ID & 0xFFFF;
            if (bone_idx <= 0 ||
                bone_idx >= (int)kDefaultBoneNames.size()) continue;

            const std::string& fbx_name = kDefaultBoneNames[bone_idx];

            // Map FBX name → canonical name
            auto it = self->cfg_.bone_name_map.find(fbx_name);
            if (it == self->cfg_.bone_name_map.end()) continue;
            const std::string& canonical = it->second;

            BodyData bd;
            bd.position = Eigen::Vector3d(rb.x, rb.y, rb.z);
            bd.rot_wxyz = Eigen::Vector4d(rb.qw, rb.qx, rb.qy, rb.qz);
            raw.body_data[canonical] = bd;
        }
    }

    if (!raw.body_data.empty())
        self->queue_.push(std::move(raw));
}

} // namespace gmr
