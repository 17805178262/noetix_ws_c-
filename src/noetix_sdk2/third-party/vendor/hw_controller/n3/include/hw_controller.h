#pragma once 

#include <atomic>
#include <memory>
#include <string>

#include "common_type.h"

namespace legged {
struct ImuRawData {
  float acc[3];
  float gyro[3];
  float quat[4];
};

class Imu;
class HwController {

public:
    HwController(HwController& other) = delete;  
    void  operator=(const HwController&) = delete;
    
    virtual ~HwController();

    static HwController* GetInstance();  

    std::string GetVersion();

    bool CreateCanu(std::string name, uint8_t ethercat_id);

    bool AttachActuator(std::string canu_name, CtrlChannel ch, ActuatorType type,
                      std::string actuator_name, uint8_t can_id);

    bool Init(std::string ifname, uint64_t cycle_ns, bool enable_dc);

    
public:
    bool EnableAllActuator();
    bool EnableActuator(const std::string& name);
    bool SetZeroPosition(const std::string& name);

    bool DisableAllActuator();
    bool DisableActuator(const std::string& name);

    // float GetTempure(const std::string& name);
    // ActautorState GetPowerState(const std::string& name);
    // ActautorMode GetMode(const std::string& name);
    std::string GetErrorString(const std::string& name);
    int GetTemperature(const std::string& name);
    float GetEffort(const std::string& name);
    float GetVelocity(const std::string& name);
    float GetPosition(const std::string& name);
    
    bool GetAllEnableState();

    //void SetMitParam(const std::string& name, MitParam param);

    void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp, float kd);
    void SetHeartCnt(const uint64_t heart_cnt);

    // 新增：将 IMU 挂载到指定的 Canu 上
    bool AttachImu(std::string canu_name, std::string imu_name);
     // 新增：获取指定 Canu 上的 IMU 指针
    Imu* GetImu(const std::string& canu_name); 
    bool GetImuData(const std::string& canu_name, ImuRawData& data);
    void rt_ethercat_run();
    void rt_ethercat_get_data();
    void rt_ethercat_set_command(); 


protected:
    HwController();

private:
  std::atomic_bool is_running_{false};
  static HwController* instance_;
};

using HwControllerPtr = std::shared_ptr<HwController>;

}






