#ifndef NOETIX_HARDWARE_BUMI_BACKEND_HPP_
#define NOETIX_HARDWARE_BUMI_BACKEND_HPP_

#include <memory>
#include <vector>

#include "noetix/hardware/hardware_backend.hpp"

namespace noetix {

/** Bumi EtherCAT backend. Exposes logical ankle pitch/roll in the common slots. */
class BumiBackend final : public HardwareBackend {
 public:
  BumiBackend();
  ~BumiBackend() override;

  bool configure(const RobotConfig& config) override;
  bool start() override;
  bool stop() override;
  bool read(std::vector<ActuatorState>& state) override;
  bool readImu(ImuState& imu) override;
  bool write(const std::vector<ActuatorCmd>& command) override;
  bool enableActuators() override;
  bool disableActuators() override;
  bool actuatorsEnabled() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace noetix

#endif  // NOETIX_HARDWARE_BUMI_BACKEND_HPP_
