#ifndef NOETIX_HARDWARE_BACKEND_HPP_
#define NOETIX_HARDWARE_BACKEND_HPP_

#include <vector>
#include "noetix/config/robot_config.hpp"
#include "noetix/hardware/actuator_cmd.hpp"
#include "noetix/hardware/actuator_state.hpp"
#include "noetix/robot/imu_state.hpp"

namespace noetix {

/**
 * @brief 硬件后端抽象接口
 * 
 * 本接口定义了与机器人底层硬件（执行器、传感器等）通信的标准方法。
 * 具体实现类（N3、Bumi）应继承此类
 * 并实现所有纯虚函数，以提供统一的硬件访问能力。
 * 
 * 生命周期管理：
 * 1. 构造对象后，调用 configure() 进行初始化配置
 * 2. 调用 start() 启动硬件通信
 * 3. 周期性调用 read() 和 write() 进行数据交互（通常在实时循环中）
 * 4. 调用 stop() 安全关闭硬件
 * 
 * @note 本接口不保证线程安全，调用者应确保 read/write 在实时上下文中
 *       被正确同步（通常由 PeriodicThread 驱动）。
 * @warning read() 和 write() 应在确定性的时间周期内执行，避免阻塞。
 */
class HardwareBackend {
 public:
  /**
   * @brief 虚析构函数，确保派生类资源正确释放
   */
  virtual ~HardwareBackend() = default;

  /**
   * @brief 配置硬件后端
   * 
   * 该函数应完成硬件初始化参数设置，例如：
   * - 打开通信设备（网卡、串口等）
   * - 建立执行器拓扑映射（关节名 ↔ 总线 ID）
   * - 加载执行器限幅参数（位置/速度/力矩范围）
   * - 分配内部缓冲区
   * 
   * @param config 包含所有必要配置信息的结构体
   * @return true  配置成功，硬件已准备就绪可调用 start()
   * @return false 配置失败（如无效参数、设备不存在等）
   * 
   * @note 本函数应在程序启动时调用一次，不应在运行时重复调用（除非需要重新初始化）。
   */
  virtual bool configure(const RobotConfig& config) = 0;

  /**
   * @brief 启动硬件通信
   * 
   * 该函数应激活硬件设备（如使能执行器、启动 EtherCAT 主站调度），
   * 使 read() 和 write() 能够正常收发数据。
   * 
   * @return true  启动成功，硬件已进入运行状态
   * @return false 启动失败（如设备未响应、使能超时）
   */
  virtual bool start() = 0;

  /**
   * @brief 停止硬件通信
   * 
   * 该函数应安全关闭硬件设备（如禁用执行器、关闭通信接口），
   * 释放占用的系统资源。调用后 read/write 不应再被调用。
   * 
   * @return true  停止成功
   * @return false 停止失败（可忽略，通常用于错误恢复）
   */
  virtual bool stop() = 0;

  /**
   * @brief 从硬件读取所有执行器的状态
   * 
   * 该函数从底层总线获取最新的传感器数据（位置、速度、力矩、温度等），
   * 并填充到 state 向量中。向量大小应已在 configure 阶段确定为 dof。
   * 
   * @param[out] state 用于存放所有执行器状态的向量，调用前应确保尺寸正确
   * @return true  数据读取成功，state 内容有效
   * @return false 读取失败（通信错误），state 内容未更新（保留上一次有效值）
   * 
   * @note 通常在实时循环的起点调用（rt_ethercat_get_data 阶段）。
   *       若发生错误，应尽快返回 false 并记录日志，避免影响控制循环。
   */
  virtual bool read(std::vector<ActuatorState>& state) = 0;

  /**
   * @brief 读取当前 IMU 状态。
   *
   * 默认返回 false，保证没有 IMU 的硬件 backend 不需要立即修改。
   * 实现 IMU 的 backend 应返回与最近一次 read() 同一硬件周期的数据。
   */
  virtual bool readImu(ImuState& imu)
  {
    (void)imu;
    return false;
  }

  /**
   * @brief 返回执行器是否已被底层确认全部使能。
   *
   * 默认 false，避免不支持使能反馈的 backend 被误报为 enabled。
   */
  virtual bool enableActuators() { return false; }

  virtual bool disableActuators() { return false; }

  virtual bool actuatorsEnabled() const
  {
    return false;
  }

  /**
   * @brief 向硬件写入所有执行器的控制指令
   * 
   * 该函数将 command 向量中的每个执行器指令（目标位置、速度、力矩、kp/kd）
   * 打包并通过总线发送到底层驱动器。
   * 
   * @param command 包含所有执行器控制指令的向量，尺寸应与 dof 一致
   * @return true  指令发送成功
   * @return false 发送失败（总线错误）
   * 
   * @note 通常在实时循环的终点调用（rt_ethercat_set_command 阶段）。
   *       发送失败时，可能需在下一周期重试或触发紧急停止。
   */
  virtual bool write(const std::vector<ActuatorCmd>& command) = 0;
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_BACKEND_HPP_