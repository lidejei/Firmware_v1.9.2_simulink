//
// File: PID_types.h
//
// Code generated for Simulink model 'PID'.
//
// Model version                  : 1.63
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Aug 26 14:11:53 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_PID_types_h_
#define RTW_HEADER_PID_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_Reference_
#define DEFINED_TYPEDEF_FOR_Reference_

typedef struct {
  real32_T phi_ref_rad;
  real32_T theta_ref_rad;
  real32_T psi_ref_rad;
  real32_T p_ref_radDs;
  real32_T q_ref_radDs;
  real32_T r_ref_radDs;
} Reference;

#endif

#ifndef DEFINED_TYPEDEF_FOR_States_
#define DEFINED_TYPEDEF_FOR_States_

typedef struct {
  real32_T phi_rad;
  real32_T theta_rad;
  real32_T psi_rad;
  real32_T p_radDs;
  real32_T q_radDs;
  real32_T r_radDs;
} States;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Command_
#define DEFINED_TYPEDEF_FOR_Command_

typedef struct {
  uint8_T reset;
  uint8_T mode;
  real32_T base_throttle;
} Command;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Model_Param_T_
#define DEFINED_TYPEDEF_FOR_Model_Param_T_

typedef struct {
  real32_T Kp_phi;
  real32_T Kp_theta;
  real32_T Kp_psi;
  real32_T Kp_p;
  real32_T Kp_q;
  real32_T Kp_r;
  real32_T Ki_p;
  real32_T Ki_q;
  real32_T Ki_r;
  real32_T Ki_Max;
  real32_T Ki_Min;
  real32_T Kd_p;
  real32_T Kd_q;
  real32_T Kd_r;
  real32_T Kd_Max;
  real32_T Kd_Min;
} Model_Param_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Control_Output_
#define DEFINED_TYPEDEF_FOR_Control_Output_

typedef struct {
  real32_T u_x;
  real32_T u_y;
  real32_T u_z;
} Control_Output;

#endif

// Forward declaration for rtModel
typedef struct tag_RTM_PID_T RT_MODEL_PID_T;

#endif                                 // RTW_HEADER_PID_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
