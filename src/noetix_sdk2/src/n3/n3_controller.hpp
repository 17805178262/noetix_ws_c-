#ifndef NOETIX_HARDWARE_N3_CONTROLLER_HPP_
#define NOETIX_HARDWARE_N3_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "noetix/robot/imu_state.hpp"

namespace noetix
{

/**
 * @brief N3 vendor HwController isolation wrapper.
 *
 * Vendor cycle semantics used by the original N3HW implementation:
 *
 *   read side:
 *     SetHeartCnt()
 *     rt_ethercat_get_data()
 *     GetPosition/GetVelocity/GetEffort()
 *
 *   write side:
 *     SetMitCmd()
 *     rt_ethercat_set_command()
 *     rt_ethercat_run()
 *
 * rt_ethercat_run() is the physical EtherCAT PDO TX/RX step.
 */
class N3Controller
{
public:
    N3Controller();
    ~N3Controller();

    N3Controller(const N3Controller&) = delete;
    N3Controller& operator=(const N3Controller&) = delete;

    bool init(const std::string& ifname,
              std::uint64_t cycle_time_ns,
              bool enable_dc);

    bool createCanu(const std::string& name,
                    int ecat_id);

    bool attachActuator(const std::string& canu_name,
                        int channel,
                        const std::string& type,
                        const std::string& actuator_name,
                        int can_id);

    /** Attach the IMU configured on a CANU before HwController::Init(). */
    bool attachImu(const std::string& canu_name,
                   const std::string& imu_name);

    /** Read the IMU sample already decoded by rt_ethercat_get_data(). */
    bool getImuData(const std::string& canu_name,
                    ImuState& imu) const;

    bool enableAll();
    bool disableAll();
    bool allActuatorsEnabled() const;

    int slaveCount() const;
    bool allSlavesOperational() const;

    void setHeartCnt(std::uint64_t count);

    /** Decode/synchronize the previously received PDO to CANU/actuators. */
    void fetchData();

    /** Encode CANU/actuator commands into the EtherCAT output PDO. */
    void stageCommand();

    /** Execute one physical EtherCAT PDO exchange. */
    void runCycle();

    double getPosition(const std::string& name) const;
    double getVelocity(const std::string& name) const;
    double getEffort(const std::string& name) const;
    double getTemperature(const std::string& name) const;
    std::string getErrorString(const std::string& name) const;

    void setMitCmd(const std::string& name,
                   double pos,
                   double vel,
                   double tau_ff,
                   double kp,
                   double kd);

private:
    class Impl;
    Impl* impl_{nullptr};
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_N3_CONTROLLER_HPP_
