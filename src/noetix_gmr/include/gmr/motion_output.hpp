#pragma once
#include "robot_presets.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace gmr {
// Transport-independent encoder. The ROS message carries these float values directly.
class MotionOutput {
public:
    struct Config {
        RobotPreset robot;
        double pelvis_z_offset = 0.0;
        double lin_vel_alpha = 1.0;
        double ang_vel_alpha = 1.0;
        double lin_vel_max = 0.0;
        double ang_vel_max = 0.0;
    };
    explicit MotionOutput(Config cfg) : cfg_(std::move(cfg)) {
        if (cfg_.robot.num_joints <= 0 ||
            cfg_.robot.joint_ids_map.size() != size_t(cfg_.robot.num_joints))
            throw std::invalid_argument("Invalid joint reorder map");
        std::vector<bool> used(cfg_.robot.num_joints, false);
        for (int id : cfg_.robot.joint_ids_map) {
            if (id < 0 || id >= cfg_.robot.num_joints || used[id])
                throw std::invalid_argument("Joint reorder map must be a permutation");
            used[id] = true;
        }
        for (double a : {cfg_.lin_vel_alpha, cfg_.ang_vel_alpha})
            if (!std::isfinite(a) || a < 0 || a > 1)
                throw std::invalid_argument("Velocity alphas must be in [0,1]");
        for (double m : {cfg_.lin_vel_max, cfg_.ang_vel_max})
            if (!std::isfinite(m) || m < 0)
                throw std::invalid_argument("Velocity limits must be nonnegative");
        if (!std::isfinite(cfg_.pelvis_z_offset))
            throw std::invalid_argument("Invalid pelvis offset");
    }
    void reset() { has_previous_ = false; velocity_initialized_ = false; }

    std::optional<std::vector<float>> encode(const Eigen::VectorXd& qpos,
                                             double frame_time, double relative_time) {
        if (qpos.size() != 7 + cfg_.robot.num_joints || !qpos.allFinite() ||
            !std::isfinite(frame_time) || !std::isfinite(relative_time)) return std::nullopt;
        if (has_previous_ && frame_time < previous_time_ - 1e-6) return std::nullopt;
        Eigen::Vector3d pos = qpos.head<3>();
        pos.z() += cfg_.pelvis_z_offset;
        Eigen::Quaterniond quat(qpos[3], qpos[4], qpos[5], qpos[6]);
        if (quat.norm() < 1e-8) return std::nullopt;
        quat.normalize();
        const bool fresh = !has_previous_ || frame_time > previous_time_ + 1e-6;
        if (has_previous_ && fresh) {
            const double dt = frame_time - previous_time_;
            const Eigen::Vector3d lv = (pos - previous_pos_) / dt;
            Eigen::Quaterniond delta = quat * previous_quat_.conjugate();
            // q 与 -q 表示相同姿态，使用最短旋转，避免符号切换产生接近 2π/dt 的尖峰。
            delta.normalize();
            if (delta.w() < 0) delta.coeffs() *= -1;
            const double s = delta.vec().norm();
            const double angle = 2.0 * std::atan2(s, delta.w());
            const Eigen::Vector3d av = s > 1e-12 ?
                Eigen::Vector3d(delta.vec() * (angle / (s * dt))) : Eigen::Vector3d::Zero();
            const bool spike = (cfg_.lin_vel_max > 0 && lv.norm() > cfg_.lin_vel_max) ||
                               (cfg_.ang_vel_max > 0 && av.norm() > cfg_.ang_vel_max);
            if (!spike) {
                if (!velocity_initialized_) { lin_vel_ = lv; ang_vel_ = av; }
                else {
                    lin_vel_ = cfg_.lin_vel_alpha * lv + (1 - cfg_.lin_vel_alpha) * lin_vel_;
                    ang_vel_ = cfg_.ang_vel_alpha * av + (1 - cfg_.ang_vel_alpha) * ang_vel_;
                }
                velocity_initialized_ = true;
            }
        }
        std::vector<float> out(14 + cfg_.robot.num_joints);
        out[0] = static_cast<float>(relative_time);
        for (int i = 0; i < 3; ++i) out[1+i] = static_cast<float>(pos[i]);
        out[4] = static_cast<float>(quat.w()); out[5] = static_cast<float>(quat.x());
        out[6] = static_cast<float>(quat.y()); out[7] = static_cast<float>(quat.z());
        for (int i = 0; i < 3; ++i) {
            out[8+i] = velocity_initialized_ ? static_cast<float>(lin_vel_[i]) : 0.0f;
            out[11+i] = velocity_initialized_ ? static_cast<float>(ang_vel_[i]) : 0.0f;
        }
        for (int i = 0; i < cfg_.robot.num_joints; ++i)
            out[14+i] = static_cast<float>(qpos[7+cfg_.robot.joint_ids_map[i]]);
        for (float v : out) if (!std::isfinite(v)) { reset(); return std::nullopt; }
        if (fresh) {
            previous_time_ = frame_time; previous_pos_ = pos; previous_quat_ = quat;
            has_previous_ = true;
        }
        return out;
    }
private:
    Config cfg_;
    bool has_previous_ = false, velocity_initialized_ = false;
    double previous_time_ = 0;
    Eigen::Vector3d previous_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond previous_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d lin_vel_ = Eigen::Vector3d::Zero(), ang_vel_ = Eigen::Vector3d::Zero();
};
} // namespace gmr
