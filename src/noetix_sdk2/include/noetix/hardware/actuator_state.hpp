#ifndef NOETIX_HARDWARE_ACTUATOR_STATE_HPP_
#define NOETIX_HARDWARE_ACTUATOR_STATE_HPP_

#include <cstdint>

namespace noetix {

/**
 * @brief 单执行器实时状态反馈（低层接口）
 * 
 * 该结构体封装了从硬件总线（如 CAN/EtherCAT）读取的单个关节/执行器
 * 的实时运行数据，包括位置、速度、力矩、温度和错误码。
 * 
 * 所有物理量均采用国际单位制（SI）：
 * - 位置：rad
 * - 速度：rad/s
 * - 力矩：N·m
 * - 温度：℃
 * 
 * @note 该数据通常在实时控制循环的 read() 阶段填充，并传递给上层控制器
 *       作为反馈输入（如用于状态估计、阻抗控制、安全监控等）。
 */
struct ActuatorState {
  float position{0.0F};      ///< 当前关节位置 [rad]
  float velocity{0.0F};      ///< 当前关节速度 [rad/s]（正值表示正方向旋转）
  float torque{0.0F};        ///< 当前实际输出力矩 [N·m]（由电流环或力矩传感器估算得到）
  float temperature{0.0F};   ///< 电机绕组或驱动器的温度 [℃]（用于过热保护）
  std::uint32_t error{0};    ///< 错误标志位（bitmask），0 表示无错误，各 bit 含义由具体驱动器手册定义
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_ACTUATOR_STATE_HPP_