/**
 * fzmotion_reader.cpp
 */
#include "fzmotion_reader.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace gmr {

// FZMotion outputs more bones than the existing FBX IK configs need. This map
// keeps the same canonical names used by OptiTrackReader / fbx_to_*.json.
const std::map<std::string, std::string>
FzmotionReader::kDefaultFzToCanonical = {
    {"Hips",          "Hips"},
    {"Neck",          "Neck"},
    {"Head",          "Head"},
    {"LeftShoulder",  "LeftShoulder"},
    {"LeftArm",       "LeftArm"},
    {"LeftForeArm",   "LeftForeArm"},
    {"LeftHand",      "LeftHand"},
    {"RightShoulder", "RightShoulder"},
    {"RightArm",      "RightArm"},
    {"RightForeArm",  "RightForeArm"},
    {"RightHand",     "RightHand"},
    {"LeftUpLeg",     "LeftUpLeg"},
    {"LeftLeg",       "LeftLeg"},
    {"LeftFoot",      "LeftFoot"},
    {"RightUpLeg",    "RightUpLeg"},
    {"RightLeg",      "RightLeg"},
    {"RightFoot",     "RightFoot"},

    // FZMotion naming differs slightly from the FBX names used by existing configs.
    {"Spine1",        "Spine"},
    {"Chest",         "Spine1"},
    {"LeftToe",       "LeftToeBase"},
    {"RightToe",      "RightToeBase"},
};

FzmotionReader::FzmotionReader(FrameQueue& queue, Config cfg)
    : BaseReader(queue), cfg_(std::move(cfg))
{
    if (cfg_.bone_name_map.empty())
        cfg_.bone_name_map = kDefaultFzToCanonical;
}

FzmotionReader::~FzmotionReader() {
    disconnect();
}

void FzmotionReader::connect() {
    if (cfg_.server_ip.empty())
        throw std::runtime_error("[FzmotionReader] server_ip is empty");

    sdk_ = lusternet::getFZReceive();
    if (!sdk_)
        throw std::runtime_error("[FzmotionReader] getFZReceive() returned null");

    sdk_->Init();
    sdk_->Connect(cfg_.server_ip);

    for (int i = 0; i < 50; ++i) {
        if (sdk_->IsConnected()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!sdk_->IsConnected())
        throw std::runtime_error("[FzmotionReader] failed to connect to "
                                 + cfg_.server_ip);

    connected_ = true;
    stop_flag_ = false;
    reader_thread_ = std::thread(&FzmotionReader::readerLoop, this);

    if (cfg_.verbose) {
        std::cout << "[FzmotionReader] Connected. server=" << cfg_.server_ip
                  << " mapped_bones=" << cfg_.bone_name_map.size()
                  << " scale=" << cfg_.position_scale;
        if (!cfg_.subject_filter.empty())
            std::cout << " subject='" << cfg_.subject_filter << "'";
        std::cout << "\n";
    }
}

void FzmotionReader::disconnect() {
    stop_flag_ = true;
    if (reader_thread_.joinable()) reader_thread_.join();

    if (sdk_) {
        try {
            sdk_->Disconnect(cfg_.server_ip);
            sdk_->Close();
        } catch (...) {
            // Some SDK builds throw during close; shutdown should remain best-effort.
        }
        connected_ = false;
        if (cfg_.verbose)
            std::cout << "[FzmotionReader] Disconnected.\n";
    }
    sdk_.reset();
}

void FzmotionReader::readerLoop() {
    lusternet::LusterMocapData mocap;
    const double scale = cfg_.position_scale;

    // FZMotion is Y-up; the existing IK configs are aligned to the Z-up pipeline.
    static const Eigen::Quaterniond R_yup_to_zup(
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
    static const Eigen::Quaterniond R_yup_to_zup_inv = R_yup_to_zup.conjugate();

    bool have_frame = false;
    uint32_t last_frame = 0;
    while (!stop_flag_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (!sdk_ || !sdk_->IsConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        try {
            mocap = lusternet::LusterMocapData{};
            sdk_->ReceiveData(mocap, /*RecState=*/1);
        } catch (const std::exception& e) {
            std::cerr << "[FzmotionReader] ReceiveData threw: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (stop_flag_) break;
        if (mocap.FrameBodysPose.empty()) continue;
        if (have_frame && mocap.FrameID == last_frame) continue;
        have_frame = true;
        last_frame = mocap.FrameID;

        const lusternet::LST_BODY_DATA* picked = nullptr;
        for (const auto& body : mocap.FrameBodysPose) {
            if (!body.IsTrack) continue;
            if (!cfg_.subject_filter.empty() &&
                body.BodyName != cfg_.subject_filter) continue;
            picked = &body;
            break;
        }
        if (!picked) continue;

        RawFrame raw;
        raw.frame_number = ++frame_counter_;
        raw.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        for (const auto& joint : picked->vecJointNodes) {
            auto it = cfg_.bone_name_map.find(joint.sJointName);
            if (it == cfg_.bone_name_map.end()) continue;
            const std::string& canonical = it->second;

            Eigen::Vector3d p_yup(joint.X, joint.Y, joint.Z);
            Eigen::Quaterniond q_yup(joint.qw, joint.qx, joint.qy, joint.qz);

            Eigen::Vector3d p_zup = R_yup_to_zup * p_yup;
            Eigen::Quaterniond q_zup = R_yup_to_zup * q_yup * R_yup_to_zup_inv;

            BodyData bd;
            bd.position = p_zup * scale;
            bd.rot_wxyz = Eigen::Vector4d(q_zup.w(), q_zup.x(), q_zup.y(), q_zup.z());
            raw.body_data[canonical] = bd;
        }

        if (!raw.body_data.empty())
            queue_.push(std::move(raw));
    }
}

} // namespace gmr
