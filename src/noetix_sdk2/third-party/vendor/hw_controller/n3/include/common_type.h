#ifndef N3_COMMON_TYPE_H
#define N3_COMMON_TYPE_H

#include <cmath>
#include <cstdint>
#include <string>

// ==================== 数学常量 ====================
/**
 * @def M_2PI
 * @brief 2π 的浮点常量
 */
#define M_2PI (2.0f * M_PI)

/**
 * @def M_4PI
 * @brief 4π 的浮点常量
 */
#define M_4PI (4.0f * M_PI)

namespace legged {

// ==================== 枚举类型定义 ====================

/**
 * @brief CAN 总线通道编号
 * 
 * 每个 CANU（CAN 转 EtherCAT 网关）设备最多支持 3 路 CAN 通道，
 * 每个通道可挂载多个执行器（电机）。
 */
enum class CtrlChannel : uint8_t {
  CTRL_CH1 = 0,  ///< 通道 1
  CTRL_CH2 = 1,  ///< 通道 2
  CTRL_CH3 = 2,  ///< 通道 3
};

/**
 * @brief 执行器（电机）型号枚举
 * 
 * 不同型号的电机具有不同的物理参数，包括：
 * - 额定扭矩 / 峰值扭矩
 * - 最高转速
 * - 转子惯量
 * - 编码器分辨率
 * - 温度传感器类型等
 * 
 * 本枚举用于在系统中标识电机型号，以便自动加载对应的参数限幅。
 */
enum class ActuatorType {
  ACTR_8522,    ///< 型号 8522（大扭矩，类似 4315，扭矩 150 N·m）
  ACTR_5014,    ///< 型号 5014（中扭矩 54 N·m）
  ACTR_4308,    ///< 型号 4308（小扭矩 22 N·m）
  UNKOWN,       ///< 未知型号（用于错误处理或未识别的字符串）
};

/**
 * @brief 将字符串转换为 ActuatorType 枚举
 * 
 * @param type 型号名称字符串（例如 "ACTR_8522"）
 * @return 对应的枚举值；若无法匹配任何已知型号，则返回 ActuatorType::UNKOWN
 * 
 * @note 此函数主要用于从配置文件（如 YAML）中解析字符串形式的型号名称。
 */
static ActuatorType StringToType(std::string type) {
  if (type == "ACTR_8522") return ActuatorType::ACTR_8522;
  if (type == "ACTR_5014") return ActuatorType::ACTR_5014;
  if (type == "ACTR_4308") return ActuatorType::ACTR_4308;
  return ActuatorType::UNKOWN;
}

/**
 * @brief 执行器状态（从站状态机）
 * 
 * 对应驱动器内部的状态控制字，用于控制执行器的使能/去使能、
 * 错误清除和校准操作。
 * 
 * @note 这些状态通常通过总线（CAN/EtherCAT）写入执行器的
 *       控制寄存器。
 */
enum ActautorState : uint8_t {
  STATE_DISABLE = 0,      ///< 去使能状态（电机处于安全模式，不输出力矩）
  STATE_ENABLE = 1,       ///< 使能状态（电机可以响应控制指令）
  STATE_CLEAR_ERROR = 2,  ///< 清除错误状态（复位故障标志）
  STATE_CALIBRATION = 3,  ///< 校准模式（执行器寻找机械零点或进行初始标定）
};

/**
 * @brief 执行器控制模式
 * 
 * 决定驱动器如何解释下发的指令数据，不同的模式对应不同的
 * 控制目标（电流/力矩、速度、位置）以及是否启用斜坡滤波。
 * 
 * @note MIT 模式为专有模式，同时接受位置、速度、前馈力矩
 *       及刚度/阻尼参数，实现阻抗控制。
 */
enum ActautorMode : uint8_t {
  MODE_CURRENT = 0,        ///< 直接电流（力矩）控制模式
  MODE_CURRENT_RAMP = 1,   ///< 电流（力矩）斜坡模式，带缓启动滤波
  MODE_VELOCITY = 2,       ///< 速度控制模式
  MODE_VELOCITY_RAMP = 3,  ///< 速度斜坡模式，带加减速平滑
  MODE_POSITION = 4,       ///< 位置控制模式（通常带内部 PID）
  MODE_POSITION_RAMP = 5,  ///< 位置斜坡模式，带运动规划滤波
  MODE_MIT = 6,            ///< MIT 阻抗控制模式（位置/速度/刚度/阻尼联合控制）
};

// ==================== MIT 控制参数结构体 ====================

/**
 * @brief MIT 控制模式下各物理量的限幅参数
 * 
 * 该结构体定义了执行器在 MIT 阻抗控制模式下的安全运行范围，
 * 包括位置、速度、力矩以及刚度（kp）和阻尼（kd）的上下限。
 * 
 * 当上层控制器下发指令超出这些限幅时，底层驱动会自动截断
 * 到安全范围内，以保护硬件。
 * 
 * @note 所有物理量使用国际单位制（SI）：
 *       - 位置：弧度（rad）
 *       - 速度：弧度/秒（rad/s）
 *       - 力矩：牛顿·米（N·m）
 *       - 刚度（kp）：N·m/rad
 *       - 阻尼（kd）：N·m·s/rad
 */
struct MitParam {
  double pos_min = 0.0;  ///< 位置下限 [rad]
  double pos_max = 0.0;  ///< 位置上限 [rad]
  double vel_min = 0.0;  ///< 速度下限 [rad/s]
  double vel_max = 0.0;  ///< 速度上限 [rad/s]
  double toq_min = 0.0;  ///< 力矩下限 [N·m]（注意变量名为 toq，为 torque 的简写）
  double toq_max = 0.0;  ///< 力矩上限 [N·m]
  double kp_min = 0.0;   ///< 刚度下限 [N·m/rad]
  double kp_max = 0.0;   ///< 刚度上限 [N·m/rad]
  double kd_min = 0.0;   ///< 阻尼下限 [N·m·s/rad]
  double kd_max = 0.0;   ///< 阻尼上限 [N·m·s/rad]
};

// ==================== 各型号电机的默认 MIT 参数限幅宏 ====================

/**
 * @def M_8522_MIT_MODE_DEFAULT_PARAM
 * @brief 型号 8522 电机的默认 MIT 控制参数限幅
 * 
 * 该电机为大扭矩型号，适用于关节负载较大的应用。
 * - 位置范围：±12.5 rad
 * - 速度范围：±18 rad/s
 * - 力矩范围：±150 N·m
 * - 刚度范围：0 ~ 500 N·m/rad
 * - 阻尼范围：0 ~ 50 N·m·s/rad
 */
#define M_8522_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,               \
      .pos_max = 12.5,                \
      .vel_min = -18.0,               \
      .vel_max = 18.0,                \
      .toq_min = -150.0,              \
      .toq_max = 150.0,               \
      .kp_min = 0.0,                  \
      .kp_max = 500.0,                \
      .kd_min = 0.0,                  \
      .kd_max = 50.0,                 \
  }

/**
 * @def M_5014_MIT_MODE_DEFAULT_PARAM
 * @brief 型号 5014 电机的默认 MIT 控制参数限幅
 * 
 * 该电机为中等扭矩型号。
 * - 位置范围：±12.5 rad
 * - 速度范围：±14 rad/s
 * - 力矩范围：±54 N·m
 * - 刚度范围：0 ~ 500 N·m/rad
 * - 阻尼范围：0 ~ 50 N·m·s/rad
 */
#define M_5014_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,               \
      .pos_max = 12.5,                \
      .vel_min = -14.0,               \
      .vel_max = 14.0,                \
      .toq_min = -54.0,               \
      .toq_max = 54.0,                \
      .kp_min = 0.0,                  \
      .kp_max = 500.0,                \
      .kd_min = 0.0,                  \
      .kd_max = 50.0,                 \
  }

/**
 * @def M_4308_MIT_MODE_DEFAULT_PARAM
 * @brief 型号 4308 电机的默认 MIT 控制参数限幅
 * 
 * 该电机为小扭矩型号，适用于轻负载或高速应用。
 * - 位置范围：±12.5 rad
 * - 速度范围：±16 rad/s
 * - 力矩范围：±22 N·m
 * - 刚度范围：0 ~ 500 N·m/rad
 * - 阻尼范围：0 ~ 50 N·m·s/rad
 */
#define M_4308_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,               \
      .pos_max = 12.5,                \
      .vel_min = -16.0,               \
      .vel_max = 16.0,                \
      .toq_min = -22.0,               \
      .toq_max = 22.0,                \
      .kp_min = 0.0,                  \
      .kp_max = 500.0,                \
      .kd_min = 0.0,                  \
      .kd_max = 50.0,                 \
  }

/**
 * @def M_RS05_MIT_MODE_DEFAULT_PARAM
 * @brief 型号 RS05 电机（小型）的默认 MIT 控制参数限幅
 * 
 * RS05 为微型电机，适用于末端执行器或精密定位。
 * - 位置范围：±12.5 rad
 * - 速度范围：±50 rad/s（高速）
 * - 力矩范围：±5.5 N·m（小扭矩）
 * - 刚度范围：0 ~ 500 N·m/rad
 * - 阻尼范围：0 ~ 5 N·m·s/rad（较低阻尼上限）
 */
#define M_RS05_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,               \
      .pos_max = 12.5,                \
      .vel_min = -50.0,               \
      .vel_max = 50.0,                \
      .toq_min = -5.5,                \
      .toq_max = 5.5,                 \
      .kp_min = 0.0,                  \
      .kp_max = 500.0,                \
      .kd_min = 0.0,                  \
      .kd_max = 5.0,                  \
  }

/**
 * @def M_R86_MIT_MODE_DEFAULT_PARAM
 * @brief 型号 R86 电机的默认 MIT 控制参数限幅
 * 
 * R86 为低速大扭矩电机，适用于高负载关节。
 * - 位置范围：±12.5 rad
 * - 速度范围：±2 rad/s（极低速）
 * - 力矩范围：±150 N·m（大扭矩）
 * - 刚度范围：0 ~ 500 N·m/rad
 * - 阻尼范围：0 ~ 5 N·m·s/rad
 */
#define M_R86_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,               \
      .pos_max = 12.5,                \
      .vel_min = -2.0,                \
      .vel_max = 2.0,                 \
      .toq_min = -150.0,              \
      .toq_max = 150.0,               \
      .kp_min = 0.0,                  \
      .kp_max = 500.0,                \
      .kd_min = 0.0,                  \
      .kd_max = 5.0,                  \
  }

/**
 * @def M_R28_MIT_MODE_DEFAULT_PARAM
 * @brief 型号 R28 电机的默认 MIT 控制参数限幅
 * 
 * R28 同样为低速大扭矩电机，参数与 R86 相同。
 * - 位置范围：±12.5 rad
 * - 速度范围：±2 rad/s
 * - 力矩范围：±150 N·m
 * - 刚度范围：0 ~ 500 N·m/rad
 * - 阻尼范围：0 ~ 5 N·m·s/rad
 */
#define M_R28_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,               \
      .pos_max = 12.5,                \
      .vel_min = -2.0,                \
      .vel_max = 2.0,                 \
      .toq_min = -150.0,              \
      .toq_max = 150.0,               \
      .kp_min = 0.0,                  \
      .kp_max = 500.0,                \
      .kd_min = 0.0,                  \
      .kd_max = 5.0,                  \
  }

}  // namespace N3

#endif  // N3_COMMON_TYPE_H