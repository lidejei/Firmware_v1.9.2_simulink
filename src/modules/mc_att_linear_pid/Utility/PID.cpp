//
// File: PID.cpp
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
#include "PID.h"
#include "PID_private.h"

const Control_Output PID_rtZControl_Output = {
  0.0F,                                // u_x
  0.0F,                                // u_y
  0.0F                                 // u_z
} ;                                    // Control_Output ground

// Exported block parameters
/*  real32_T Kp_phi;	roll kp
  real32_T Kp_theta;	pitch kp
  real32_T Kp_psi;		yaw kp
  real32_T Kp_p;		roll vel kp
  real32_T Kp_q;		pitch vel kp
  real32_T Kp_r;		yaw vel kp
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
  */
Model_Param_T Model_Param = {
	4.0F,
	4.0F,
	1.8F,
	0.1F,
	0.1F,
	0.1F,
	0.02F,
	0.02F,
	0.02F,
	0.2F,
	-0.2F,
	0.0002F,
	0.0002F,
	0.0002F,
	0.2F,
	-0.2F
} ;                                    // Variable: Model_Param
                                       //  Referenced by:
                                       //    '<S7>/gain1' 
                                       //    '<S7>/gain2'
                                       //    '<S7>/gain3'
                                       //    '<S7>/Saturation'
                                       //    '<S8>/gain1'
                                       //    '<S8>/gain2'
                                       //    '<S8>/gain3'
                                       //    '<S8>/Saturation'
                                       //    '<S9>/gain1'
                                       //    '<S9>/gain2'
                                       //    '<S9>/gain3'
                                       //    '<S14>/Gain'
                                       //    '<S14>/Gain1'


// Model step function
void PIDModelClass::step()
{
  real32_T rtb_rate_err_raw_idx_0;
  real32_T rtb_rate_err_raw_idx_1;
  real32_T rtb_rate_err_raw_idx_2;
  real32_T rtb_Sum_idx_0;
  real32_T rtb_Sum_idx_1;
  real32_T rtb_Sum_idx_2;
  real32_T u0;
  real32_T u0_0;
  real32_T u0_1;
  real32_T tmp;

  // DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/Command'

  if ((PID_U.Command_m.reset != 0) || (PID_DW.DiscreteTimeIntegrator_PrevRese !=
       0)) {
    PID_DW.DiscreteTimeIntegrator_DSTATE[0] = PID_ConstB.Constant[0];
    PID_DW.DiscreteTimeIntegrator_DSTATE[1] = PID_ConstB.Constant[1];
    PID_DW.DiscreteTimeIntegrator_DSTATE[2] = PID_ConstB.Constant[2];
  }

  // Switch: '<S3>/Switch' incorporates:
  //   Gain: '<S14>/Gain'
  //   Gain: '<S14>/Gain1'
  //   Gain: '<S14>/Gain2'
  //   Inport: '<Root>/Command'
  //   Inport: '<Root>/Reference'
  //   Inport: '<Root>/States'
  //   Sum: '<S11>/Sum'
  //   Trigonometry: '<S14>/Tanh'
  //   Trigonometry: '<S14>/Tanh1'

  if (PID_U.Command_m.mode > 0) {
    rtb_rate_err_raw_idx_0 = PID_U.States_i.p_radDs;//current roll vel
    rtb_rate_err_raw_idx_1 = PID_U.States_i.q_radDs;//pitch
    rtb_rate_err_raw_idx_2 = PID_U.States_i.r_radDs;//yaw
  } else {
    rtb_rate_err_raw_idx_0 = std::tanh((PID_U.Reference_e.phi_ref_rad -
      PID_U.States_i.phi_rad) * Model_Param.Kp_phi);
    rtb_rate_err_raw_idx_1 = std::tanh((PID_U.Reference_e.theta_ref_rad -
      PID_U.States_i.theta_rad) * Model_Param.Kp_phi);
    rtb_rate_err_raw_idx_2 = std::tanh((PID_U.Reference_e.psi_ref_rad -
      PID_U.States_i.psi_rad) * Model_Param.Kp_psi);
  }

  // End of Switch: '<S3>/Switch'

  // Sum: '<S4>/Sum' incorporates:
  //   Inport: '<Root>/States'

  rtb_rate_err_raw_idx_0 -= PID_U.States_i.p_radDs;
  rtb_rate_err_raw_idx_1 -= PID_U.States_i.q_radDs;
  rtb_rate_err_raw_idx_2 -= PID_U.States_i.r_radDs;

  // DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' incorporates:
  //   Inport: '<Root>/Command'

  if ((PID_U.Command_m.reset != 0) || (PID_DW.DiscreteTimeIntegrator1_PrevRes !=
       0)) {
    PID_DW.DiscreteTimeIntegrator1_DSTATE[0] = 0.0F;
    PID_DW.DiscreteTimeIntegrator1_DSTATE[1] = 0.0F;
    PID_DW.DiscreteTimeIntegrator1_DSTATE[2] = 0.0F;
  }

  // Gain: '<S10>/k_pt' incorporates:
  //   DiscreteIntegrator: '<S10>/Discrete-Time Integrator1'
  //   Sum: '<S10>/Sum5'

  rtb_Sum_idx_0 = (rtb_rate_err_raw_idx_0 -
                   PID_DW.DiscreteTimeIntegrator1_DSTATE[0]) * 10.0F;
  rtb_Sum_idx_1 = (rtb_rate_err_raw_idx_1 -
                   PID_DW.DiscreteTimeIntegrator1_DSTATE[1]) * 10.0F;
  rtb_Sum_idx_2 = (rtb_rate_err_raw_idx_2 -
                   PID_DW.DiscreteTimeIntegrator1_DSTATE[2]) * 10.0F;

  // Product: '<S7>/Multiply' incorporates:
  //   Constant: '<S7>/gain1'

  u0 = Model_Param.Kd_p * rtb_Sum_idx_0;

  // Saturate: '<S8>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'

  if (PID_DW.DiscreteTimeIntegrator_DSTATE[0] > Model_Param.Ki_Max) {
    tmp = Model_Param.Ki_Max;
  } else if (PID_DW.DiscreteTimeIntegrator_DSTATE[0] < Model_Param.Ki_Min) {
    tmp = Model_Param.Ki_Min;
  } else {
    tmp = PID_DW.DiscreteTimeIntegrator_DSTATE[0];
  }

  // Saturate: '<S7>/Saturation'
  if (u0 > Model_Param.Kd_Max) {
    u0 = Model_Param.Kd_Max;
  } else {
    if (u0 < Model_Param.Kd_Min) {
      u0 = Model_Param.Kd_Min;
    }
  }

  // Sum: '<S5>/Add' incorporates:
  //   Constant: '<S9>/gain1'
  //   Product: '<S9>/Multiply'

  u0 += Model_Param.Kp_p * rtb_rate_err_raw_idx_0 + tmp;

  // Product: '<S7>/Multiply' incorporates:
  //   Constant: '<S7>/gain2'

  u0_0 = Model_Param.Kd_q * rtb_Sum_idx_1;

  // Saturate: '<S8>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'

  if (PID_DW.DiscreteTimeIntegrator_DSTATE[1] > Model_Param.Ki_Max) {
    tmp = Model_Param.Ki_Max;
  } else if (PID_DW.DiscreteTimeIntegrator_DSTATE[1] < Model_Param.Ki_Min) {
    tmp = Model_Param.Ki_Min;
  } else {
    tmp = PID_DW.DiscreteTimeIntegrator_DSTATE[1];
  }

  // Saturate: '<S7>/Saturation'
  if (u0_0 > Model_Param.Kd_Max) {
    u0_0 = Model_Param.Kd_Max;
  } else {
    if (u0_0 < Model_Param.Kd_Min) {
      u0_0 = Model_Param.Kd_Min;
    }
  }

  // Sum: '<S5>/Add' incorporates:
  //   Constant: '<S9>/gain2'
  //   Product: '<S9>/Multiply'

  u0_0 += Model_Param.Kp_q * rtb_rate_err_raw_idx_1 + tmp;

  // Product: '<S7>/Multiply' incorporates:
  //   Constant: '<S7>/gain3'

  u0_1 = Model_Param.Kd_r * rtb_Sum_idx_2;

  // Saturate: '<S8>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'

  if (PID_DW.DiscreteTimeIntegrator_DSTATE[2] > Model_Param.Ki_Max) {
    tmp = Model_Param.Ki_Max;
  } else if (PID_DW.DiscreteTimeIntegrator_DSTATE[2] < Model_Param.Ki_Min) {
    tmp = Model_Param.Ki_Min;
  } else {
    tmp = PID_DW.DiscreteTimeIntegrator_DSTATE[2];
  }

  // Saturate: '<S7>/Saturation'
  if (u0_1 > Model_Param.Kd_Max) {
    u0_1 = Model_Param.Kd_Max;
  } else {
    if (u0_1 < Model_Param.Kd_Min) {
      u0_1 = Model_Param.Kd_Min;
    }
  }

  // Sum: '<S5>/Add' incorporates:
  //   Constant: '<S9>/gain3'
  //   Product: '<S9>/Multiply'

  u0_1 += Model_Param.Kp_r * rtb_rate_err_raw_idx_2 + tmp;

  // Saturate: '<S5>/Saturation'
  if (u0 > 1.0F) {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_x = 1.0F;
  } else if (u0 < -1.0F) {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_x = -1.0F;
  } else {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_x = u0;
  }

  if (u0_0 > 1.0F) {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_y = 1.0F;
  } else if (u0_0 < -1.0F) {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_y = -1.0F;
  } else {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_y = u0_0;
  }

  if (u0_1 > 1.0F) {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_z = 1.0F;
  } else if (u0_1 < -1.0F) {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_z = -1.0F;
  } else {
    // Outport: '<Root>/Control_Output'
    PID_Y.Control_Output_e.u_z = u0_1;
  }

  // End of Saturate: '<S5>/Saturation'

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S8>/gain1'
  //   Inport: '<Root>/Command'
  //   Product: '<S8>/Multiply'

  PID_DW.DiscreteTimeIntegrator_PrevRese = (int8_T)(PID_U.Command_m.reset > 0);
  PID_DW.DiscreteTimeIntegrator_DSTATE[0] += Model_Param.Ki_p *
    rtb_rate_err_raw_idx_0 * 0.004F;

  // Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1'
  PID_DW.DiscreteTimeIntegrator1_DSTATE[0] += 0.004F * rtb_Sum_idx_0;

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S8>/gain2'
  //   Product: '<S8>/Multiply'

  PID_DW.DiscreteTimeIntegrator_DSTATE[1] += Model_Param.Ki_q *
    rtb_rate_err_raw_idx_1 * 0.004F;

  // Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1'
  PID_DW.DiscreteTimeIntegrator1_DSTATE[1] += 0.004F * rtb_Sum_idx_1;

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S8>/gain3'
  //   Product: '<S8>/Multiply'

  PID_DW.DiscreteTimeIntegrator_DSTATE[2] += Model_Param.Ki_r *
    rtb_rate_err_raw_idx_2 * 0.004F;

  // Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' incorporates:
  //   Inport: '<Root>/Command'

  PID_DW.DiscreteTimeIntegrator1_DSTATE[2] += 0.004F * rtb_Sum_idx_2;
  PID_DW.DiscreteTimeIntegrator1_PrevRes = (int8_T)(PID_U.Command_m.reset > 0);
}

// Model initialize function
void PIDModelClass::initialize()
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(getRTM(), (NULL));

  // states (dwork)
  (void) memset((void *)&PID_DW, 0,
                sizeof(DW_PID_T));

  // external inputs
  (void)memset(&PID_U, 0, sizeof(ExtU_PID_T));

  // external outputs
  PID_Y.Control_Output_e = PID_rtZControl_Output;

  // InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' 
  PID_DW.DiscreteTimeIntegrator_DSTATE[0] = PID_ConstB.Constant[0];
  PID_DW.DiscreteTimeIntegrator_DSTATE[1] = PID_ConstB.Constant[1];
  PID_DW.DiscreteTimeIntegrator_DSTATE[2] = PID_ConstB.Constant[2];
  PID_DW.DiscreteTimeIntegrator_PrevRese = 0;

  // InitializeConditions for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' 
  PID_DW.DiscreteTimeIntegrator1_PrevRes = 0;
}

// Model terminate function
void PIDModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
PIDModelClass::PIDModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
PIDModelClass::~PIDModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_PID_T * PIDModelClass::getRTM()
{
  return (&PID_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
