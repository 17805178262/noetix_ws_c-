#pragma once
/**
 * body_map.hpp — Shared skeleton data types
 *
 * All readers produce a BodyMap; GMR and publishers consume it.
 * Keep this header dependency-free (Eigen only).
 */

#include <Eigen/Dense>
#include <map>
#include <string>

namespace gmr {

struct BodyData {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector4d rot_wxyz = Eigen::Vector4d(1, 0, 0, 0);  // w x y z
};

// bone_name → pose
using BodyMap = std::map<std::string, BodyData>;

} // namespace gmr
