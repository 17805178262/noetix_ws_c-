
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#define M_2PI (2.0f * M_PI)
#define M_4PI (4.0f * M_PI)

namespace legged {

enum class CtrlChannel : uint8_t {
  CTRL_CH1 = 0,
  CTRL_CH2 = 1,
  CTRL_CH3 = 2,
};

enum class ActuatorType {
  ACTR_4340,
  ACTR_4315,
  ACTR_4325,
  ACTR_RS05,
  ACTR_4340_ANK,
  ACTR_4308,
  ACTR_F5014,
  OMNI_PICKER,
  UNKOWN,
};

enum class RobotModel : uint8_t {
  BUMI1,
  BUMI2,
  UNKNOWN,
};

 struct MotorData
 {
   double pos_, vel_, tau_;                  // state
   double pos_des_, vel_des_, kp_, kd_, ff_; // command
   uint32_t error_;
   float temperature_;
 };
 
static ActuatorType StringToType(std::string type) {
  if (type == "ACTR_4340") return ActuatorType::ACTR_4340;
  if (type == "ACTR_4340_ANK") return ActuatorType::ACTR_4340_ANK;
  if (type == "ACTR_4315") return ActuatorType::ACTR_4315;
  if (type == "ACTR_4325") return ActuatorType::ACTR_4325;
  if (type == "ACTR_4308") return ActuatorType::ACTR_4308;
  if (type == "ACTR_F5014") return ActuatorType::ACTR_F5014;
  if (type == "ACTR_RS05") return ActuatorType::ACTR_RS05;
  if (type == "OMNI_PICKER") return ActuatorType::OMNI_PICKER;
  return ActuatorType::UNKOWN;
}

static RobotModel StringToRobotModel(std::string model) {
  if (model == "bumi1") return RobotModel::BUMI1;
  if (model == "bumi2") return RobotModel::BUMI2;
  return RobotModel::UNKNOWN;
}

enum ActautorState : uint8_t {
  STATE_DISABLE = 0,
  STATE_ENABLE = 1,
  STATE_CLEAR_ERROR = 2,
  STATE_CALIBRATION = 3,
};

enum ActautorMode : uint8_t {
  MODE_CURRENT = 0,
  MODE_CURRENT_RAMP = 1,
  MODE_VELOCITY = 2,
  MODE_VELOCITY_RAMP = 3,
  MODE_POSITION = 4,
  MODE_POSITION_RAMP = 5,
  MODE_MIT = 6,
};
struct ImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

struct ActautorDebug
{
   uint8_t can_err = 0;
   uint16_t actor_err = 0;
};

/**
 * @description: Mit参数
 */
struct MitParam {
  double pos_min = 0.0;
  double pos_max = 0.0;
  double vel_min = 0.0;
  double vel_max = 0.0;
  double toq_min = 0.0;
  double toq_max = 0.0;
  double kp_min = 0.0;
  double kp_max = 0.0;
  double kd_min = 0.0;
  double kd_max = 0.0;
};

#define M_4340_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -20.0,       \
      .vel_max = 20.0,        \
      .toq_min = -18.0,              \
      .toq_max = 18.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }

#define M_4340_ANK_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -20.0,       \
      .vel_max = 20.0,        \
      .toq_min = -40.0,              \
      .toq_max = 40.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }  

#define M_4315_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -18.0,       \
      .vel_max = 18.0,        \
      .toq_min = -150.0,              \
      .toq_max = 150.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }

  
#define M_4325_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -65.0,       \
      .vel_max = 65.0,        \
      .toq_min = -30.0,              \
      .toq_max = 30.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }

#define M_4308_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -16.0,       \
      .vel_max = 16.0,        \
      .toq_min = -22.0,              \
      .toq_max = 22.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 50.0,                 \
  }

#define M_F5014_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -13.5,       \
      .vel_max = 13.5,        \
      .toq_min = -55.0,              \
      .toq_max = 55.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 50.0,                 \
  }

#define M_RS05_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -50.0,       \
      .vel_max = 50.0,        \
      .toq_min = -5.5,              \
      .toq_max = 5.5,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }

#define M_R86_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -2.0,       \
      .vel_max = 2.0,        \
      .toq_min = -150.0,              \
      .toq_max = 150.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }

#define M_R28_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -12.5,       \
      .pos_max = 12.5,        \
      .vel_min = -2.0,       \
      .vel_max = 2.0,        \
      .toq_min = -150.0,              \
      .toq_max = 150.0,               \
      .kp_min = 0.0,                 \
      .kp_max = 500.0,               \
      .kd_min = 0.0,                 \
      .kd_max = 5.0,                 \
  }


}  // namespace xyber