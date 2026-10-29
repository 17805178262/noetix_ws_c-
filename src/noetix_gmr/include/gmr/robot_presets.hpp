#pragma once
#include <string>
#include <vector>
#include <stdexcept>

namespace gmr {
static constexpr int G1_NUM_JOINTS = 29;
static constexpr int G1_JOINT_IDS_MAP[G1_NUM_JOINTS] = {
     0,  6, 12,  1,  7, 13,  2,  8, 14,  3,
     9, 15, 22,  4, 10, 16, 23,  5, 11, 17,
    24, 18, 25, 19, 26, 20, 27, 21, 28
};

static constexpr int E1_NUM_JOINTS = 24;
static constexpr int E1_JOINT_IDS_MAP[E1_NUM_JOINTS] = {
     0,  6, 12,  1,  7, 13,  2,  8, 14, 19,
     3,  9, 15, 20,  4, 10, 16, 21,  5, 11,
    17, 22, 18, 23
};

static constexpr int BUMI3_NUM_JOINTS = 21;
static constexpr int BUMI3_JOINT_IDS_MAP[BUMI3_NUM_JOINTS] = {
     9, 15,  0, 10, 16,  1,  5, 11, 17,  2,  6,
    12, 18,  3,  7, 13, 19,  4,  8, 14, 20
};

// N3 V3模型按双腿、腰部、双臂和头部排列；输出仍按现有GMT的左右交错顺序。
// 索引来自新包joint_order.json，腕部保持yaw、pitch、roll的传输语义。
static constexpr int N3_NUM_JOINTS = 29;
static constexpr int N3_JOINT_IDS_MAP[N3_NUM_JOINTS] = {
     0,  6, 12,  1,  7, 27, 13, 20,  2,  8,
    28, 14, 21,  3,  9, 15, 22,  4, 10, 16,
    23,  5, 11, 17, 24, 19, 26, 18, 25
};

struct RobotPreset {
    int              num_joints = 0;
    std::vector<int> joint_ids_map;
    double           pelvis_z_offset = -0.05;
};

inline RobotPreset presetG1() {
    RobotPreset p;
    p.num_joints       = G1_NUM_JOINTS;
    p.pelvis_z_offset  = -0.05;
    p.joint_ids_map.assign(G1_JOINT_IDS_MAP,
                           G1_JOINT_IDS_MAP + G1_NUM_JOINTS);
    return p;
}

inline RobotPreset presetE1() {
    RobotPreset p;
    p.num_joints       = E1_NUM_JOINTS;
    p.pelvis_z_offset  = 0.0;
    p.joint_ids_map.assign(E1_JOINT_IDS_MAP,
                           E1_JOINT_IDS_MAP + E1_NUM_JOINTS);
    return p;
}

inline RobotPreset presetBumi3() {
    RobotPreset p;
    p.num_joints       = BUMI3_NUM_JOINTS;
    p.pelvis_z_offset  = 0.0;
    p.joint_ids_map.assign(BUMI3_JOINT_IDS_MAP,
                           BUMI3_JOINT_IDS_MAP + BUMI3_NUM_JOINTS);
    return p;
}

inline RobotPreset presetN3() {
    RobotPreset p;
    p.num_joints       = N3_NUM_JOINTS;
    p.pelvis_z_offset  = 0.0;
    p.joint_ids_map.assign(N3_JOINT_IDS_MAP,
                           N3_JOINT_IDS_MAP + N3_NUM_JOINTS);
    return p;
}

inline RobotPreset robotPreset(const std::string& name) {
    if (name == "g1") return presetG1();
    if (name == "e1") return presetE1();
    if (name == "bumi3") return presetBumi3();
    if (name == "n3") return presetN3();
    throw std::invalid_argument("Unknown robot: " + name);
}
} // namespace gmr
