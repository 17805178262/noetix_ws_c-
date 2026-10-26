#ifndef NOETIX_ROBOT_ROBOT_STATUS_HPP_
#define NOETIX_ROBOT_ROBOT_STATUS_HPP_

#include <cstdint>

namespace noetix {

/**
 * @brief 机器人运行模式枚举
 * 
 * 定义机器人在不同运行阶段的工作模式，用于状态机管理。
 * 各模式之间存在明确的转换关系，确保系统安全可靠。
 */
enum class RobotMode : std::uint8_t {
  kUnknown = 0,   ///< 未知模式（初始状态或错误状态）
  kPassive,       ///< 被动模式（电机未使能，关节可自由运动，仅读取传感器数据）
  kStand,         ///< 站立模式（电机使能但保持零速/位置锁定，用于等待指令）
  kControl,       ///< 正常控制模式（执行上层控制算法，如行走、跳跃等）
  kEStop          ///< 紧急停止模式（立即停止所有运动，进入安全状态）
};

/**
 * @brief 机器人硬件状态摘要
 * 
 * 提供当前硬件层的整体运行状态，包括模式、连接状态、使能状态和故障码。
 * 该结构体通常由硬件接口的 status() 方法返回，用于监控和诊断。
 */
struct RobotStatus {
  RobotMode mode{RobotMode::kUnknown};  ///< 当前运行模式
  bool connected{false};                ///< 与底层硬件（如 EtherCAT 主站）的连接状态
  bool enabled{false};                  ///< 所有执行器是否均已使能（整体使能标志）
  std::uint32_t fault{0};              ///< 故障码（bitmask），0 表示无故障，各 bit 含义由具体实现定义
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_ROBOT_STATUS_HPP_