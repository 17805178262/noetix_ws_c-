#ifndef NOETIX_ROBOT_INTERFACE_HPP_
#define NOETIX_ROBOT_INTERFACE_HPP_

#include "noetix/config/robot_config.hpp"
#include "noetix/robot/low_cmd.hpp"
#include "noetix/robot/low_state.hpp"
#include "noetix/robot/robot_status.hpp"

namespace noetix {

/**
 * @brief 机器人硬件抽象接口类
 *
 * 本接口定义了机器人控制软件与底层硬件（如电机驱动器、传感器等）
 * 之间的标准通信协议。所有具体的机器人实现（如四足机器人、双足机器人、
 * 机械臂等）都应继承此类并实现其纯虚函数。
 *
 * 设计理念：
 * - 生命周期管理：通过 configure() → start() → (read/write 循环) → stop()
 *   来管理硬件连接和运行状态。
 * - 数据分离：LowState 承载传感器数据（从硬件读入），LowCmd 承载控制指令（写出到硬件）。
 * - 线程安全：本接口不保证线程安全，调用者应在合适的线程上下文（如实时控制循环）中
 *   调用 read/write，并确保 configure/start/stop 不发生并发调用。
 */
class RobotInterface {
 public:
  /**
   * @brief 虚析构函数（默认实现）
   *
   * 确保通过基类指针删除派生类对象时，能正确调用派生类的析构函数，
   * 从而安全释放资源。同时保持移动构造/赋值的自动生成。
   */
  virtual ~RobotInterface() = default;

  /**
   * @brief 配置机器人硬件参数
   *
   * 该函数应完成硬件的初始化参数配置，例如：
   * - 设置通信接口（EtherCAT/CAN 设备名、波特率等）
   * - 加载执行器型号对应的限幅参数（位置/速度/力矩 min/max）
   * - 建立硬件拓扑映射（关节名称 ↔ 总线 ID）
   *
   * 注意：此函数通常在程序启动时调用一次，不应在运行时重复调用，
   * 除非需要重新初始化硬件。
   *
   * @param config 包含所有必要硬件配置信息的结构体
   * @return true  配置成功，硬件已就绪可调用 start()
   * @return false 配置失败，错误信息可通过日志或状态获取
   */
  virtual bool configure(const RobotConfig& config) = 0;

  /**
   * @brief 启动硬件通信与控制循环
   *
   * 该函数应激活硬件设备（如使能驱动器、启动 EtherCAT 主站调度），
   * 使 read() 和 write() 能够正常收发数据。
   *
   * 启动成功后，外部控制线程将周期性调用 read() 和 write()。
   *
   * @return true  启动成功，硬件已进入运行状态
   * @return false 启动失败（如设备未响应、使能超时）
   */
  virtual bool start() = 0;

  /**
   * @brief 停止硬件通信与控制循环
   *
   * 该函数应安全地关闭硬件设备（如禁用驱动器、停止 EtherCAT 通信），
   * 释放占用的资源。调用后 read/write 不应再被调用（或调用无效果）。
   *
   * 通常用于程序退出或紧急停止场景。
   *
   * @return true  停止成功
   * @return false 停止失败（部分设备未正常关闭，可重试）
   */
  virtual bool stop() = 0;

  /**
   * @brief 从硬件读取传感器数据
   *
   * 该函数应从底层通信总线获取最新一轮的反馈数据（电机位置、速度、
   * 力矩、IMU 状态等），并填充到 LowState 结构中。
   *
   * 调用约定：
   * - 应在控制循环的起点调用，获取当前周期的“测量值”。
   * - 调用频率应与控制周期一致（如 1kHz）。
   * - 若硬件通信超时或数据校验失败，应返回 false 并保留上一次有效状态。
   *
   * @param[out] state 用于存放读取到的状态数据
   * @return true  数据读取成功，state 内容有效
   * @return false 读取失败（通信错误），state 内容未更新
   */
  virtual bool read(LowState& state) = 0;

  /**
   * @brief 向硬件发送控制指令
   *
   * 该函数应将 LowCmd 中的控制指令（如目标位置、速度、力矩、
   * 刚度/阻尼系数等）通过通信总线下发到对应的执行器。
   *
   * 调用约定：
   * - 应在控制循环的终点调用，将计算好的指令发送出去。
   * - 调用频率应与控制周期一致。
   * - 若硬件通信阻塞或发送失败，应返回 false，但调用者通常不应重试，
   *   而是在下一周期继续尝试。
   *
   * @param command 包含所有关节控制指令的结构体
   * @return true  指令发送成功
   * @return false 发送失败（总线错误）
   */
  virtual bool write(const LowCmd& command) = 0;

  /** Runtime drive enable/disable. Communication remains started. */
  virtual bool enableActuators() = 0;
  virtual bool disableActuators() = 0;
  virtual bool actuatorsEnabled() const = 0;

  /**
   * @brief 获取当前机器人硬件状态
   *
   * 返回一个描述当前硬件运行状态的对象，包括：
   * - 是否已配置（configured）
   * - 是否已启动（started）
   * - 是否存在错误（error flags）
   * - 可能的通信健康状态等
   *
   * 该函数通常用于监控或调试，不应影响控制实时性。
   *
   * @return RobotStatus 当前硬件状态摘要
   */
  virtual RobotStatus status() const = 0;
};

}  // namespace noetix

#endif  // NOETIX_ROBOT_INTERFACE_HPP_