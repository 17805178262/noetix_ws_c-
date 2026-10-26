#ifndef NOETIX_HARDWARE_BUMI_CONTROLLER_HPP_
#define NOETIX_HARDWARE_BUMI_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "noetix/robot/imu_state.hpp"

namespace noetix {

/** Thin isolation wrapper around the Bumi vendor HwController ABI. */
class BumiController {
 public:
  BumiController();
  ~BumiController();

  BumiController(const BumiController&) = delete;
  BumiController& operator=(const BumiController&) = delete;

  bool createCanu(const std::string& name, int ecat_id,
                  const std::string& robot_model);
  bool attachActuator(const std::string& canu_name, int channel,
                      const std::string& type, const std::string& actuator_name,
                      int can_id);
  bool init(const std::string& ifname, std::uint64_t cycle_time_ns, bool enable_dc);

  bool enableAll();
  bool disableAll();
  bool allActuatorsEnabled() const;

  void setHeartCnt(std::uint64_t count);
  void fetchData();
  void stageCommand();
  void runCycle();

  double getPosition(const std::string& name) const;
  double getVelocity(const std::string& name) const;
  double getEffort(const std::string& name) const;
  bool getImuData(const std::string& canu_name, ImuState& imu) const;
  void setMitCmd(const std::string& name, double pos, double vel,
                 double tau_ff, double kp, double kd);

 private:
  class Impl;
  Impl* impl_{nullptr};
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_BUMI_CONTROLLER_HPP_
