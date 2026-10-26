#pragma once
#include "body_map.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gmr {
// 根据动捕帧时间判断接触；支撑期间固定脚的完整姿态，抬脚、迈步和转脚可解除锁定。
class FootContact {
public:
    struct Config {
        double height = .06;
        double enter_height = .025, exit_height = .055;
        double enter_speed = .20, exit_speed = .45;
        double release_time = .05, max_displacement = .06, max_rotation = .35;
    };

    explicit FootContact(Config config) : cfg_(config) {
        for (double value : {cfg_.height, cfg_.enter_height, cfg_.exit_height,
                             cfg_.enter_speed, cfg_.exit_speed, cfg_.release_time,
                             cfg_.max_displacement, cfg_.max_rotation})
            if (!std::isfinite(value) || value < 0)
                throw std::invalid_argument("Foot contact parameters must be finite and nonnegative");
        if (cfg_.exit_height <= cfg_.enter_height || cfg_.exit_speed <= cfg_.enter_speed ||
            cfg_.release_time <= 0 || cfg_.max_displacement <= 0 || cfg_.max_rotation <= 0)
            throw std::invalid_argument("Invalid foot contact hysteresis or release thresholds");
    }

    void apply(BodyData& body, double frame_time) {
        if (!std::isfinite(frame_time) || !body.position.allFinite() ||
            !body.rot_wxyz.allFinite() || body.rot_wxyz.norm() < 1e-8)
            throw std::invalid_argument("Invalid foot contact input");
        BodyData measured = body;
        measured.rot_wxyz.normalize();
        // 首帧预热重复求解同一帧，不能据此制造速度或推进接触状态。
        if (have_previous_ && frame_time == previous_time_) {
            body = planted_ ? anchor_ : measured;
            return;
        }
        const double dt = frame_time - previous_time_;
        if (!have_previous_ || dt <= 0) {
            have_previous_ = false;
            planted_ = false;
            moving_time_ = 0;
            velocity_.setZero();
        } else if (dt > .2) {
            // 断流只重置速度差分；恢复后仍在支撑阈值内的脚保留原锚点，避免按噪声逐次滑动。
            // 真正抬脚、转脚或位置重置仍由下方几何条件解除锁定。
            moving_time_ = 0;
            velocity_.setZero();
        } else {
            // 只平滑接触检测速度，避免相邻帧反向噪声反复触发离地。
            const double alpha = dt / (.04 + dt);
            velocity_ = (1 - alpha) * velocity_ + alpha * (measured.position - previous_) / dt;
        }

        const double height = measured.position.z() - cfg_.height;
        bool released = false;
        if (planted_) {
            Eigen::Quaterniond q(measured.rot_wxyz[0], measured.rot_wxyz[1],
                                 measured.rot_wxyz[2], measured.rot_wxyz[3]);
            Eigen::Quaterniond anchor(anchor_.rot_wxyz[0], anchor_.rot_wxyz[1],
                                      anchor_.rot_wxyz[2], anchor_.rot_wxyz[3]);
            const bool fast = velocity_.head<2>().norm() > cfg_.exit_speed ||
                              velocity_.z() > cfg_.exit_speed;
            moving_time_ = fast ? moving_time_ + dt : 0;
            released = height > cfg_.exit_height || moving_time_ >= cfg_.release_time ||
                       (measured.position - anchor_.position).norm() > cfg_.max_displacement ||
                       q.angularDistance(anchor) > cfg_.max_rotation;
            if (released) planted_ = false;
        }
        // 本帧刚解除接触时不立即重新锁定，以免滑步、转脚被吞掉。
        if (!planted_ && !released && height <= cfg_.enter_height &&
            (!have_previous_ || (velocity_.head<2>().norm() <= cfg_.enter_speed &&
                                 std::abs(velocity_.z()) <= cfg_.enter_speed))) {
            planted_ = true;
            anchor_ = measured;
            moving_time_ = 0;
        }
        previous_ = measured.position;
        previous_time_ = frame_time;
        have_previous_ = true;
        body = planted_ ? anchor_ : measured;
    }

    bool planted() const { return planted_; }

private:
    Config cfg_;
    bool have_previous_ = false, planted_ = false;
    double previous_time_ = 0, moving_time_ = 0;
    Eigen::Vector3d previous_ = Eigen::Vector3d::Zero(), velocity_ = Eigen::Vector3d::Zero();
    BodyData anchor_;
};
} // namespace gmr
