#ifndef NOETIX_BRIDGE_ROBOT_RESOURCE_HPP_
#define NOETIX_BRIDGE_ROBOT_RESOURCE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace noetix_bridge {

/**
 * 关节资源描述：记录单个关节与其对应执行器之间的映射和校准参数。
 */
struct JointResource {
  std::string joint_name;      ///< 关节名称（唯一标识）
  std::string actuator_name;   ///< 对应的执行器名称（唯一标识）
  int direction{1};            ///< 方向系数，通常为 1 或 -1，用于修正关节与执行器的旋转方向
  double offset{0.0};          ///< 零位偏移量，单位与关节位置一致，用于校准零位
};

/**
 * 一个已配置的机器人姿态模式下完全解析后的命令条目。
 *
 * pos/vel/ff 使用 robot.yaml 中的逻辑关节坐标表示。
 * 姿态模式节点在发布 LowCmd 之前会将它们转换为执行器坐标。
 * kp/kd 是标量执行器增益，因此不进行符号翻转。
 */
struct JointModeResource {
  std::string joint_name;      ///< 关节名称（逻辑关节标识）
  std::string actuator_name;   ///< 执行器名称（物理执行器标识）
  double pos{0.0};             ///< 目标位置，逻辑关节坐标
  double vel{0.0};             ///< 目标速度，逻辑关节坐标
  double kp{0.0};              ///< 比例增益，标量执行器增益，不随方向翻转
  double kd{0.0};              ///< 微分增益，标量执行器增益，不随方向翻转
  double ff{0.0};              ///< 前馈力矩/力，逻辑关节坐标
};

/**
 * 从 config/robot.yaml 解析得到的机器人本地 LIE/STAND（躺/站）控制档位。
 *
 * joints 的顺序始终与 robot.joint_order 完全一致。
 * 一个已配置的档位必须为每个已配置的关节包含一条命令。
 */
struct JointControlProfile {
  bool configured{false};          ///< 是否已配置该档位（false 表示未启用/未配置）

  double duration_s{0.0};          ///< 该档位的持续时间，单位：秒
  double publish_hz{0.0};          ///< 控制命令的发布频率，单位：赫兹

  std::vector<JointModeResource> joints;  ///< 各关节的模式资源列表，顺序与 robot.joint_order 一致
};

/**
 * 机器人资源配置信息：汇总从配置文件中解析出的机器人相关参数与资源。
 */
struct RobotResourceInfo {
  std::string robot_root;                  ///< 机器人配置根目录（或资源根路径）
  std::string robot_yaml;                  ///< 机器人主配置文件路径（robot.yaml）
  std::vector<std::string> joint_names;    ///< 关节名称列表（按配置顺序）
  std::vector<std::string> actuator_names; ///< 执行器名称列表（按配置顺序）
  std::vector<JointResource> joints;       ///< 关节资源列表，包含关节与执行器的映射及校准参数
  std::size_t dof{0};                      ///< 自由度（Degree of Freedom），默认为 0
  std::string imu_frame{"base_imu"};       ///< IMU 坐标系名称，默认为 "base_imu"
  JointControlProfile lie_control;         ///< “躺下”（LIE）姿态的控制档位参数
  JointControlProfile stand_control;       ///< “站立”（STAND）姿态的控制档位参数
};

RobotResourceInfo loadRobotResource(const std::string& robot_name);

}  // namespace noetix_bridge
#endif
