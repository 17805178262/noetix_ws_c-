//
// File: parallel_ankle_ik_vel.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 01-Dec-2025 23:06:58
//
#ifndef PARALLEL_ANKLE_IK_VEL_H
#define PARALLEL_ANKLE_IK_VEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void parallel_ankle_ik_vel(double theta_roll_K, double theta_pitch_K,
  double theta_1_K, double theta_2_K, double dtheta_roll_K, double
  dtheta_pitch_K, double which_leg, double is_ankle_sys, double *dtheta_1_K,
  double *dtheta_2_K);

#endif

//
// File trailer for parallel_ankle_ik_vel.h
//
// [EOF]
//
