#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
namespace gmr {
static Eigen::VectorXd tuneOutputQpos(const Eigen::VectorXd& input,
                                      double motion_scale,
                                      double root_z_offset)
{
    Eigen::VectorXd output = input;
    if (output.size() > 2)
        output[2] += root_z_offset;
    // MuJoCo free joint occupies qpos[0..6]; robot hinge joints start at 7.
    for (Eigen::Index i = 7; i < output.size(); ++i)
        output[i] *= motion_scale;
    return output;
}

class JointSpeedLimiter {
public:
    JointSpeedLimiter(double motion_scale,
                      double root_z_offset,
                      double root_xy_scale,
                      double max_joint_speed,
                      bool lock_root_tilt)
        : motion_scale_(motion_scale),
          root_z_offset_(root_z_offset),
          root_xy_scale_(root_xy_scale),
          max_joint_speed_(max_joint_speed),
          lock_root_tilt_(lock_root_tilt)
    {}

    void reset() { initialized_ = false; }

    Eigen::VectorXd apply(const Eigen::VectorXd& input)
    {
        Eigen::VectorXd output = tuneOutputQpos(
            input, motion_scale_, root_z_offset_);
        applyRootTiltLock(output);
        const auto now = std::chrono::steady_clock::now();

        if (!initialized_ || previous_.size() != output.size()) {
            if (output.size() > 1)
                root_xy_origin_ = output.head<2>();
            applyRootXyScale(output);
            previous_ = output;
            last_update_ = now;
            initialized_ = true;
            return output;
        }

        applyRootXyScale(output);

        const double dt = std::clamp(
            std::chrono::duration<double>(now - last_update_).count(),
            0.001, 0.1);
        last_update_ = now;

        if (max_joint_speed_ > 0.0) {
            const double max_delta = max_joint_speed_ * dt;
            for (Eigen::Index i = 7; i < output.size(); ++i) {
                const double delta = std::clamp(
                    output[i] - previous_[i], -max_delta, max_delta);
                output[i] = previous_[i] + delta;
            }
        }

        previous_ = output;
        return output;
    }

private:
    void applyRootTiltLock(Eigen::VectorXd& output) const
    {
        if (!lock_root_tilt_ || output.size() < 7) return;

        Eigen::Quaterniond q(output[3], output[4], output[5], output[6]);
        if (q.norm() < 1e-8) return;
        q.normalize();
        const double yaw = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        output[3] = std::cos(0.5 * yaw);
        output[4] = 0.0;
        output[5] = 0.0;
        output[6] = std::sin(0.5 * yaw);
    }

    void applyRootXyScale(Eigen::VectorXd& output) const
    {
        if (output.size() > 1) {
            output[0] = root_xy_origin_[0]
                      + (output[0] - root_xy_origin_[0]) * root_xy_scale_;
            output[1] = root_xy_origin_[1]
                      + (output[1] - root_xy_origin_[1]) * root_xy_scale_;
        }
    }

    double motion_scale_ = 1.0;
    double root_z_offset_ = 0.0;
    double root_xy_scale_ = 1.0;
    double max_joint_speed_ = 0.0;
    bool lock_root_tilt_ = false;
    bool initialized_ = false;
    Eigen::Vector2d root_xy_origin_ = Eigen::Vector2d::Zero();
    Eigen::VectorXd previous_;
    std::chrono::steady_clock::time_point last_update_;
};

}
