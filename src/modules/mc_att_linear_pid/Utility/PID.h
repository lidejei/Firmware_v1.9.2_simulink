//
// File: PID.h
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
#ifndef RTW_HEADER_PID_h_
#define RTW_HEADER_PID_h_
#include <cmath>
#include <string.h>
#include <stddef.h>
#ifndef PID_COMMON_INCLUDES_
# define PID_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // PID_COMMON_INCLUDES_

#include "PID_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (default storage) for system '<Root>'
typedef struct {
  real32_T DiscreteTimeIntegrator_DSTATE[3];// '<S8>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator1_DSTATE[3];// '<S10>/Discrete-Time Integrator1' 
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S8>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator1_PrevRes;// '<S10>/Discrete-Time Integrator1'
} DW_PID_T;

// Invariant block signals (default storage)
typedef const struct tag_ConstB_PID_T {
  real32_T Constant[3];                // '<S8>/Constant'
} ConstB_PID_T;

// External inputs (root inport signals with default storage)
typedef struct {
  Reference Reference_e;               // '<Root>/Reference'
  States States_i;                     // '<Root>/States'
  Command Command_m;                   // '<Root>/Command'
} ExtU_PID_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  Control_Output Control_Output_e;     // '<Root>/Control_Output'
} ExtY_PID_T;

// Real-time Model Data Structure
struct tag_RTM_PID_T {
  const char_T * volatile errorStatus;
};

// External data declarations for dependent source files
extern const Control_Output PID_rtZControl_Output;// Control_Output ground
extern const ConstB_PID_T PID_ConstB;  // constant block i/o

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern Model_Param_T Model_Param;      // Variable: Model_Param
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


// Class declaration for model PID
class PIDModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_PID_T PID_U;

  // External outputs
  ExtY_PID_T PID_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  PIDModelClass();

  // Destructor
  ~PIDModelClass();

  // Real-Time Model get method
  RT_MODEL_PID_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_PID_T PID_DW;

  // Real-Time Model
  RT_MODEL_PID_T PID_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Kp_phi ' : Unused code path elimination
//  Block '<S13>/Kp_psi' : Unused code path elimination
//  Block '<S13>/Kp_theta' : Unused code path elimination
//  Block '<S13>/Multiply' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PID'
//  '<S1>'   : 'PID/Inner_Loop'
//  '<S2>'   : 'PID/Outter_Loop'
//  '<S3>'   : 'PID/Reference_Mapping'
//  '<S4>'   : 'PID/Inner_Loop/Error'
//  '<S5>'   : 'PID/Inner_Loop/PID_Controller'
//  '<S6>'   : 'PID/Inner_Loop/Error/PT1_Filter1'
//  '<S7>'   : 'PID/Inner_Loop/PID_Controller/D_Control'
//  '<S8>'   : 'PID/Inner_Loop/PID_Controller/I_Control'
//  '<S9>'   : 'PID/Inner_Loop/PID_Controller/P_Control'
//  '<S10>'  : 'PID/Inner_Loop/PID_Controller/D_Control/DT1_Filter'
//  '<S11>'  : 'PID/Outter_Loop/Error'
//  '<S12>'  : 'PID/Outter_Loop/P_Controller'
//  '<S13>'  : 'PID/Outter_Loop/P_Controller/P_Control'
//  '<S14>'  : 'PID/Outter_Loop/P_Controller/Subsystem'

#endif                                 // RTW_HEADER_PID_h_

//
// File trailer for generated code.
//
// [EOF]
//
