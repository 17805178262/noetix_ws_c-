#ifndef NOETIX_ROBOT_LOW_STATE_HPP
#define NOETIX_ROBOT_LOW_STATE_HPP

#include <cstdint>
#include <vector>
#include "noetix/robot/imu_state.hpp"
#include "noetix/robot/motor_state.hpp"
#include "noetix/robot/robot_status.hpp"

namespace noetix {

/**
 * @brief 机器人底层状态数据（从硬件读取的完整反馈）
 *
 * 该结构体封装了一个控制周期内从所有传感器和执行器采集到的完整反馈数据，
 * 包含关节状态、IMU信息、时间戳及硬件运行状态。该结构体通常由硬件接口层
 * 的 read() 函数填充，并传递给上层控制器（如MIT、RL策略）作为观测输入。
 *
 * @note 性能关键：
 *       - 所有成员数据均设计为可被高效复制（或移动）。
 *       - motor_state 使用 std::vector 存储，其大小在 configure() 阶段
 *         通过 resize() 设置，之后在实时控制循环中严禁再次调整大小，
 *         以避免动态内存分配导致的不确定延迟和实时性破坏。
 *       - 时间戳 timestamp_ns 通常由硬件时钟（如EtherCAT DC）提供，
 *         用于同步控制周期和记录数据延迟。
 *
 * @warning 实时循环（RT loop）中禁止调用 resize() 或任何可能修改
 *          motor_state 容量的操作，确保内存访问确定性。
 */
struct LowState {
  std::uint64_t timestamp_ns{0};  ///< 当前数据帧的时间戳 [纳秒]，通常源自硬件同步时钟
  std::uint64_t sequence{0};      ///< 自启动以来的帧序号（每周期递增），用于检测数据丢包或乱序
  std::vector<MotorState> motor_state;  ///< 所有关节电机的状态向量（尺寸 = 自由度数目）
  ImuState imu;                  ///< 惯性测量单元（IMU）数据，包括姿态、角速度、加速度
  RobotStatus status;            ///< 硬件当前运行状态（使能、错误、校准等标志）

  /**
   * @brief 预设电机状态向量的大小（在配置阶段调用）
   * @param dof 机器人自由度数量（即电机总数）
   *
   * @note 此函数应在 configure() 阶段调用一次，在 RT 循环中禁止调用。
   */
  void resize(std::size_t dof) { motor_state.resize(dof); }

  /**
   * @brief 获取当前电机数量（自由度）
   * @return std::size_t 电机状态向量的大小
   *
   * @note 声明为 noexcept，适合在实时循环中使用。
   */
  std::size_t dof() const noexcept { return motor_state.size(); }
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_LOW_STATE_HPP