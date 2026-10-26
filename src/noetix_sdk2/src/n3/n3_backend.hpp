#ifndef NOETIX_HARDWARE_N3_BACKEND_HPP_
#define NOETIX_HARDWARE_N3_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "noetix/hardware/hardware_backend.hpp"
#include "n3_controller.hpp"

namespace noetix
{

struct ActuatorConfig
{
    std::string name;
    std::string type;
    int channel{0};
    int can_id{0};
};

struct CanuConfig
{
    std::string name;
    int ethercat_id{0};
    std::vector<ActuatorConfig> actuators;

    bool has_imu{false};
    std::string imu_name;
};

struct EthercatConfig
{
    std::string ifname;
    std::uint64_t cycle_time_ns{1000000ULL};
    bool enable_dc{false};
};

struct N3JointCalibration
{
    int direction{1};
    double bias{0.0};
};

struct HardwareConfig
{
    EthercatConfig ethercat;
    std::vector<CanuConfig> canus;
    std::vector<std::string> actuator_order;
    std::vector<N3JointCalibration> calibration;


    // 诊断只读模式：
    // - 允许部分 actuator 无反馈时 backend 继续上线；
    // - 强制禁止 EnableAllActuator；
    // - write() 不调用 SetMitCmd，保持电机 DISABLE；
    // - 主要用于排查单条 CAN 通道、RViz、joint_states。
    bool diagnostic_read_only{false};
};

/**
 * @brief N3 real hardware backend.
 *
 * 实机坐标与旧 N3HW 对齐：
 * - 从 hardware.yaml 的 actuator_list 读取物理电机 direction/bias；
 *   robot.yaml 的关节 direction/offset 只由上层处理，避免重复换向。
 * - 复用旧 N3HW 的 Chebyshev 并联踝正解、雅可比及命令转换。
 * - 禁止在未请求 ENABLE 时调用 SetMitCmd（原厂接口可能自动使能）。
 *
 * 严格跟随原厂 N3HW 周期语义：
 *
 * read():
 *   SetHeartCnt(++heart)
 *   rt_ethercat_get_data()
 *   GetPosition/GetVelocity/GetEffort()
 *
 * controller compute
 *
 * write():
 *   SetMitCmd(...)
 *   rt_ethercat_set_command()
 *   rt_ethercat_run()
 *
 * diagnostic_read_only=true 时：
 * - 允许部分电机反馈丢失；
 * - 无效关节使用最后一次有效位置；若启动后从未有效则使用 0 rad；
 * - 无效关节速度/力矩强制为 0；
 * - 电机绝不自动 enable；
 * - 不论上层给什么命令，write() 都只发送 DISABLE 并维持 EtherCAT 通信。
 */
class N3Backend final : public HardwareBackend
{
public:
    N3Backend();
    ~N3Backend() override;

    bool configure(const RobotConfig& config) override;
    bool start() override;
    bool stop() override;

    bool read(std::vector<ActuatorState>& state) override;
    bool readImu(ImuState& imu) override;
    bool enableActuators() override;
    bool disableActuators() override;
    bool actuatorsEnabled() const override { return actuators_enabled_; }
    bool write(const std::vector<ActuatorCmd>& command) override;

private:
    struct AnkleRuntime;
    bool setupHardware();
    void queueDisable();

    /** 仅设置 q/dq/tau/kp/kd=0，不执行 EtherCAT exchange。 */
    void setZeroMitCommand();

    /**
     * 启动阶段执行一个原厂时序通信周期：
     * heartbeat -> stage output -> run EtherCAT -> decode input。
     */
    void startupCycle();

    /**
     * 刷新全部 actuator 的反馈有效性缓存。
     * @return 当前有效 actuator 数量。
     */
    std::size_t refreshFeedbackValidity(bool verbose);

    bool feedbackLooksValid(double q_motor,
                            double dq_motor,
                            double tau_motor) const;

    void dumpInvalidActuators() const;
    void dumpActuatorDiagnostics() const;

private:
    std::unique_ptr<N3Controller> controller_;
    HardwareConfig hardware_config_;

    std::size_t dof_{0};
    std::uint64_t heart_count_{0};

    bool configured_{false};
    bool started_{false};
    bool actuators_enabled_{false};

    // configure() 阶段一次性分配，RT read() 中不扩容。
    std::vector<std::uint8_t> feedback_valid_;
    std::vector<ActuatorState> last_valid_state_;

    // LowState currently exposes one primary IMU.
    // If multiple CANUs configure IMUs, the first configured IMU is used here.
    std::string primary_imu_canu_;
    std::string primary_imu_name_;
    ImuState imu_state_{};
    bool imu_configured_{false};
    std::unique_ptr<AnkleRuntime> ankle_;
    bool ankle_feedback_ready_{false};
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_N3_BACKEND_HPP_
