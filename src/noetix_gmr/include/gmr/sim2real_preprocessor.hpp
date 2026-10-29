#pragma once

#include "gmr/body_map.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gmr {

// Pico启动时标定最低点并冻结高度偏移，重连时将水平位置与偏航衔接到上一段。
class Sim2realPicoPreprocessor {
public:
    // 设置断流重连阈值、启动标定帧数和人体最低点目标高度。
    explicit Sim2realPicoPreprocessor(double reconnect_timeout = 0.5,
                                      int bootstrap_frames = 30,
                                      double minimum_height = 0.01)
        : reconnect_timeout_(reconnect_timeout),
          bootstrap_frames_(bootstrap_frames),
          minimum_height_(minimum_height) {
        if (reconnect_timeout_ <= 0.0 || bootstrap_frames_ <= 0 ||
            !std::isfinite(minimum_height_))
            throw std::invalid_argument("Invalid sim2real preprocessor configuration");
    }

    // 对一帧人体数据做坐标衔接与高度标定，不修改源时间戳。
    void process(BodyMap& bodies, double frame_time) {
        const auto pelvis = bodies.find("Pelvis");
        if (pelvis == bodies.end())
            throw std::runtime_error("sim2real preprocessing requires Pelvis");

        double raw_minimum_z = std::numeric_limits<double>::infinity();
        for (const auto& [name, body] : bodies) {
            (void)name;
            raw_minimum_z = std::min(raw_minimum_z, body.position.z());
        }
        if (!std::isfinite(raw_minimum_z))
            throw std::runtime_error("sim2real preprocessing received no finite body heights");

        const bool reconnect = initialized_ && std::isfinite(last_frame_time_) &&
            frame_time - last_frame_time_ > reconnect_timeout_;
        if (!initialized_ || reconnect) {
            source_position_ = pelvis->second.position;
            source_position_.z() = 0.0;
            source_yaw_ = yawQuaternion(pelvis->second.rot_wxyz);
            if (!initialized_) {
                destination_position_.setZero();
                destination_yaw_.setIdentity();
            } else {
                destination_position_ = tracked_position_;
                destination_yaw_ = tracked_yaw_;
            }
            initialized_ = true;
        }

        const Eigen::Quaterniond relative_yaw =
            (destination_yaw_ * source_yaw_.conjugate()).normalized();
        for (auto& [name, body] : bodies) {
            (void)name;
            body.position = destination_yaw_ *
                (source_yaw_.conjugate() * (body.position - source_position_))
                + destination_position_;
            const Eigen::Quaterniond orientation = quaternion(body.rot_wxyz);
            const Eigen::Quaterniond transformed =
                (relative_yaw * orientation).normalized();
            body.rot_wxyz =
                Eigen::Vector4d(transformed.w(), transformed.x(),
                                transformed.y(), transformed.z());
        }

        if (calibrated_frames_ < bootstrap_frames_) {
            height_offset_ = minimum_height_ - raw_minimum_z;
            ++calibrated_frames_;
        }
        for (auto& [name, body] : bodies) {
            (void)name;
            body.position.z() += height_offset_;
        }

        const auto transformed_pelvis = bodies.find("Pelvis");
        tracked_position_ = transformed_pelvis->second.position;
        tracked_position_.z() = 0.0;
        tracked_yaw_ = yawQuaternion(transformed_pelvis->second.rot_wxyz);
        last_frame_time_ = frame_time;
    }

    // 获取已完成标定的帧数。
    int calibratedFrames() const { return calibrated_frames_; }
    // 获取启动标定后固定使用的高度偏移。
    double heightOffset() const { return height_offset_; }

private:
    // 将wxyz四元数归一化。
    static Eigen::Quaterniond quaternion(const Eigen::Vector4d& value) {
        Eigen::Quaterniond q(value[0], value[1], value[2], value[3]);
        if (q.norm() < 1e-12) return Eigen::Quaterniond::Identity();
        return q.normalized();
    }

    // 从骨盆旋转中提取绕竖直轴的偏航。
    static Eigen::Quaterniond yawQuaternion(const Eigen::Vector4d& value) {
        const Eigen::Matrix3d rotation = quaternion(value).toRotationMatrix();
        const double yaw = std::atan2(rotation(1,0), rotation(0,0));
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    double reconnect_timeout_;
    int bootstrap_frames_;
    double minimum_height_;
    bool initialized_ = false;
    int calibrated_frames_ = 0;
    double height_offset_ = 0.0;
    double last_frame_time_ = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d source_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond source_yaw_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d destination_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond destination_yaw_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d tracked_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond tracked_yaw_ = Eigen::Quaterniond::Identity();
};

}  // namespace gmr
