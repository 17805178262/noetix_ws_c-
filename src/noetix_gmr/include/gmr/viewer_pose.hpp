#pragma once
#include "robot_presets.hpp"
#include <cmath>
#include <vector>

namespace gmr {
// Undo MotionOutput's controller-order mapping, preserving the published root pose.
inline bool decodeViewerPose(const std::vector<float>& frame, const RobotPreset& robot,
                             std::vector<double>& qpos) {
    if (frame.size() != size_t(14 + robot.num_joints) ||
        robot.joint_ids_map.size() != size_t(robot.num_joints)) return false;
    for (float value : frame) if (!std::isfinite(value)) return false;
    double norm = 0;
    for (int i=4; i<8; ++i) norm += double(frame[i]) * frame[i];
    norm = std::sqrt(norm);
    if (norm < 1e-8) return false;
    std::vector<double> pose(7 + robot.num_joints);
    for (int i=0; i<3; ++i) pose[i] = frame[1+i];
    for (int i=0; i<4; ++i) pose[3+i] = frame[4+i] / norm;
    std::vector<bool> used(robot.num_joints, false);
    for (int i=0; i<robot.num_joints; ++i) {
        const int index = robot.joint_ids_map[i];
        if (index < 0 || index >= robot.num_joints || used[index]) return false;
        used[index] = true;
        pose[7+index] = frame[14+i];
    }
    qpos = std::move(pose);
    return true;
}
} // namespace gmr
