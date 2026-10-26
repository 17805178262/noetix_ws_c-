#pragma once
/**
 * pico_reader.hpp — XRoboToolkit (Pico 4 Ultra) reader
 *
 * Directly ports py_bindings.cpp logic into a standalone C++ class.
 * No xrobotoolkit_sdk.h needed — only PXREARobotSDK.h + nlohmann/json.
 *
 * Data flow:
 *   PXREAInit callback (PXREADeviceStateJson)
 *     → parse stateJson JSON
 *     → extract Body.joints[i].p = "x,y,z,qx,qy,qz,qw"
 *     → Unity → right-hand coord transform
 *     → map Pico bone name → canonical IK config name
 *     → push RawFrame into FrameQueue
 */

#include "base_reader.hpp"
#include <nlohmann/json.hpp>
#include <PXREARobotSDK.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <map>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace gmr {

class PicoReader : public BaseReader {
public:
    struct Config {
        std::map<std::string, std::string> bone_name_map;  // empty = use default
        bool normalize_initial_yaw = false;
        double startup_timeout_sec = 30.0;
    };

    PicoReader(FrameQueue& queue, Config cfg);
    ~PicoReader() override;

    void connect()    override;
    void disconnect() override;
    std::string name() const override { return "Pico"; }
    bool isConnected() const override { return connected_; }

    int64_t latestBodyStampNs() const {
        std::lock_guard<std::mutex> lk(body_mtx_);
        return body_stamp_ns_;
    }

    bool isBodyDataAvailable() const {
        std::lock_guard<std::mutex> lk(body_mtx_);
        return body_available_;
    }

private:
    void onStateJson(const char* json_str);

    static void pxreaCallback(
        void* ctx, PXREAClientCallbackType type, int status, void* user_data);


    static void transformUnityToRH(
        const std::array<double,7>& pose,   // x y z qx qy qz qw (Unity)
        Eigen::Vector3d& pos_out,
        Eigen::Vector4d& quat_wxyz_out);

    void applyInitialYawNormalization(BodyMap& data);

    void fetchLoop();

    Config              cfg_;
    std::atomic<bool>   connected_{false};
    std::atomic<bool> service_connected_{false};
    std::atomic<uint64_t> devices_found_{0}, devices_lost_{0}, state_frames_{0}, malformed_frames_{0};
    bool                sdk_initialized_ = false;
    bool                yaw_captured_ = false;
    Eigen::Quaterniond  yaw_inverse_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d     yaw_origin_ = Eigen::Vector3d::Zero();

    mutable std::mutex                   body_mtx_;
    std::array<std::array<double,7>,24>  body_joints_{};
    int64_t                              body_stamp_ns_   = 0;
    bool                                 body_available_  = false;
    int64_t                              last_stamp_ns_   = 0;

    std::thread          fetch_thread_;
    std::atomic<bool>    fetch_running_{false};

    // Unity→RH: R = [[1,0,0],[0,0,-1],[0,1,0]] as quaternion wxyz
    static constexpr double kRw =  0.70710678;
    static constexpr double kRx = -0.70710678;
    static constexpr double kRy =  0.0;
    static constexpr double kRz =  0.0;

    static const std::vector<std::string>        kBoneNames;
    static const std::map<std::string,std::string> kDefaultMap;
};

} // namespace gmr
