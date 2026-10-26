#ifndef NOETIX_ROBOT_IMU_STATE_HPP
#define NOETIX_ROBOT_IMU_STATE_HPP

#include <array>

namespace noetix {

/**
 * @brief IMU（惯性测量单元）状态数据
 * 
 * 该结构体封装了从 IMU 传感器读取的实时姿态和运动数据，包括：
 * - 姿态四元数（表示传感器相对于世界坐标系的旋转）
 * - 三轴角速度（陀螺仪测量值）
 * - 三轴加速度（加速度计测量值）
 * 
 * 所有数据均采用国际单位制（SI）：
 * - 角度：使用四元数（无量纲）
 * - 角速度：弧度/秒（rad/s）
 * - 加速度：米/秒²（m/s²）
 * 
 * 坐标系约定：
 * - 采用右手坐标系，通常为：X 向前，Y 向左，Z 向上（ENU 或 NED 取决于具体传感器配置）
 * - 四元数存储顺序为 w, x, y, z，其中 w 为实部
 * 
 * @note 该数据通常由硬件驱动在实时循环中填充，作为上层控制算法（如状态估计器、平衡控制器）的输入。
 */
struct ImuState {
  /**
   * @brief 姿态四元数，表示从 IMU 坐标系到世界坐标系的旋转
   * 
   * 存储顺序：[w, x, y, z]，其中 w 为实部。
   * 默认值为单位四元数 (1, 0, 0, 0)，表示无旋转。
   */
  std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F, 0.0F};

  /**
   * @brief 三轴角速度，由陀螺仪测量 [rad/s]
   * 
   * 顺序：[x, y, z]，分别对应绕 X、Y、Z 轴的旋转速率。
   * 正值表示沿对应轴的正方向旋转（遵循右手定则）。
   */
  std::array<float, 3> gyroscope{};

  /**
   * @brief 三轴加速度，由加速度计测量 [m/s²]
   * 
   * 顺序：[x, y, z]，分别对应沿 X、Y、Z 轴的加速度分量。
   * 包含重力加速度（静态时，Z 轴通常约为 +9.81 m/s² 或 -9.81 m/s²，
   * 取决于传感器朝向和坐标系定义）。
   */
  std::array<float, 3> accelerometer{};

  /**
   * @brief 当前 IMU 样本是否有效。
   *
   * N3 backend 只有在 vendor GetImuData() 成功且四元数/惯性数据均为有限值时
   * 才置为 true。其他 backend 如果尚未实现 IMU，可保持默认 false。
   */
  bool valid{false};
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_IMU_STATE_HPP