#ifndef _CTRL_OBJ_H_
#define _CTRL_OBJ_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file   modules/ctrl/src/32b/ctrl_obj.h
//! \brief Defines the structures for the CTRL object 
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"
#include "IQmathLib.h"

#include "ctrl_states.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "motor.h"
#include "offset.h"
#include "pid.h"
#include "est.h"
#include "svgen.h"
#include "traj.h"



//!
//!
//! \defgroup CTRL_OBJ CTRL_OBJ
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief  Defines the maximum number of controllers that are instantiated
//!         into ROM
//!
#define CTRL_NUM_CONTROLLERS       (2)


// **************************************************************************
// the typedefs

////! \brief Enumeration for the controller states
////!
//typedef enum {
//  CTRL_State_Error=0,           //!< the controller error state
//  CTRL_State_Idle,              //!< the controller idle state
//  CTRL_State_OffLine,           //!< the controller offline state
//  CTRL_State_OnLine,            //!< the controller online state
//  CTRL_numStates                //!< the number of controller states
//} CTRL_State_e;


//! \brief Enumeration for the error codes
//!
typedef enum
{
  CTRL_ErrorCode_NoError=0,        //!< no error error code
  CTRL_ErrorCode_IdClip,           //!< Id clip error code
  CTRL_ErrorCode_EstError,         //!< estimator error code
  CTRL_numErrorCodes               //!< the number of controller error codes
} CTRL_ErrorCode_e;


//! \brief Enumeration for the target processors
//!
typedef enum
{
  CTRL_TargetProc_2806x=0,   //!< 2806x processor
  CTRL_TargetProc_2805x,     //!< 2805x processor
  CTRL_TargetProc_2803x,     //!< 2803x processor
  CTRL_TargetProc_2802x,     //!< 2802x processor
  CTRL_TargetProc_Unknown    //!< Unknown processor
} CTRL_TargetProc_e;


//! \brief Enumeration for the controller (CTRL) types
//!
typedef enum
{
  CTRL_Type_PID_spd=0,             //!< PID Speed controller
  CTRL_Type_PID_Id,                //!< PID Id controller
  CTRL_Type_PID_Iq                 //!< PID Iq controller
} CTRL_Type_e;


//! \brief Defines the controller (CTRL) version number
//!
typedef struct _CTRL_Version_
{
  uint16_t rsvd;                 //!< reserved value
  CTRL_TargetProc_e targetProc;  //!< the target processor
  uint16_t major;                //!< the major release number
  uint16_t minor;                //!< the minor release number
} CTRL_Version;


//! \brief      Defines the controller (CTRL) object
//! \details    The controller object implements all of the FOC algorithms and the estimator
//!             functions.
//!
typedef struct _CTRL_Obj_
{
  CTRL_Version       version;                      //!< the controller version

  CTRL_State_e       state;                        //!< the current state of the controller

  CTRL_State_e       prevState;                    //!< the previous state of the controller

  CTRL_ErrorCode_e   errorCode;                    //!< the error code for the controller

  CLARKE_Handle      clarkeHandle_I;               //!< the handle for the current Clarke transform
  CLARKE_Obj         clarke_I;                     //!< the current Clarke transform object
 
  CLARKE_Handle      clarkeHandle_V;               //!< the handle for the voltage Clarke transform
  CLARKE_Obj         clarke_V;                     //!< the voltage Clarke transform object

  EST_Handle         estHandle;                    //!< the handle for the parameter estimator

  PARK_Handle        parkHandle;                   //!< the handle for the Park object
  PARK_Obj           park;                         //!< the Park transform object

  PID_Handle         pidHandle_Id;                 //!< the handle for the Id PID controller
  PID_Obj            pid_Id;                       //!< the Id PID controller object

  PID_Handle         pidHandle_Iq;                 //!< the handle for the Iq PID controller
  PID_Obj            pid_Iq;                       //!< the Iq PID controller object

  PID_Handle         pidHandle_spd;                //!< the handle for the speed PID controller
  PID_Obj            pid_spd;                      //!< the speed PID controller object

  IPARK_Handle       iparkHandle;                  //!< the handle for the inverse Park transform
  IPARK_Obj          ipark;                        //!< the inverse Park transform object

  SVGEN_Handle       svgenHandle;                  //!< the handle for the space vector generator 
  SVGEN_Obj          svgen;                        //!< the space vector generator object

  TRAJ_Handle        trajHandle_Id;                //!< the handle for the Id trajectory generator
  TRAJ_Obj           traj_Id;                      //!< the Id trajectory generator object

  TRAJ_Handle        trajHandle_spd;               //!< the handle for the speed trajectory generator
  TRAJ_Obj           traj_spd;                     //!< the speed trajectory generator object

  TRAJ_Handle        trajHandle_spdMax;            //!< the handle for the maximum speed trajectory generator
  TRAJ_Obj           traj_spdMax;                  //!< the maximum speed trajectory generator object

  MOTOR_Params       motorParams;                  //!< the motor parameters

  uint_least32_t     waitTimes[CTRL_numStates];    //!< an array of wait times for each state, estimator clock counts

  uint_least32_t     counter_state;                //!< the state counter

  uint_least16_t     numIsrTicksPerCtrlTick;       //!< Defines the number of isr clock ticks per controller clock tick

  uint_least16_t     numCtrlTicksPerCurrentTick;   //!< Defines the number of controller clock ticks per current controller clock tick

  uint_least16_t     numCtrlTicksPerSpeedTick;     //!< Defines the number of controller clock ticks per speed controller clock tick

  uint_least16_t     numCtrlTicksPerTrajTick;      //!< Defines the number of controller clock ticks per trajectory clock tick

  uint_least32_t     ctrlFreq_Hz;                  //!< Defines the controller frequency, Hz

  uint_least32_t     trajFreq_Hz;                  //!< Defines the trajectory frequency, Hz

  _iq                trajPeriod_sec;               //!< Defines the trajectory period, sec

  float_t            ctrlPeriod_sec;               //!< Defines the controller period, sec

  _iq                maxVsMag_pu;                  //!< the maximum voltage vector that is allowed, pu

  MATH_vec2          Iab_in;                       //!< the Iab input values

  MATH_vec2          Iab_filt;                     //!< the Iab filtered values

  MATH_vec2          Idq_in;                       //!< the Idq input values

  MATH_vec2          Vab_in;                       //!< the Vab input values

  _iq                spd_out;                      //!< the speed output value

  MATH_vec2          Vab_out;                      //!< the Vab output values

  MATH_vec2          Vdq_out;                      //!< the Vdq output values

  float_t            Rhf;                          //!< the Rhf value
  float_t            Lhf;                          //!< the Lhf value
  float_t            RoverL;                       //!< the R/L value

  _iq                Kp_Id;                        //!< the desired Kp_Id value
  _iq                Kp_Iq;                        //!< the desired Kp_Iq value
  _iq                Kp_spd;                       //!< the desired Kp_spd value

  _iq                Ki_Id;                        //!< the desired Ki_Id value
  _iq                Ki_Iq;                        //!< the desired Ki_Iq value
  _iq                Ki_spd;                       //!< the desired Ki_spd value

  _iq                Kd_Id;                        //!< the desired Kd_Id value
  _iq                Kd_Iq;                        //!< the desired Kd_Iq value
  _iq                Kd_spd;                       //!< the desired Kd_spd value

  _iq                Ui_Id;                        //!< the desired Ui_Id value
  _iq                Ui_Iq;                        //!< the desired Ui_Iq value
  _iq                Ui_spd;                       //!< the desired Ui_spd value

  MATH_vec2          Idq_ref;                      //!< the Idq reference values, pu

  _iq                IdRated;                      //!< the Id rated current, pu

  _iq                spd_ref;                      //!< the speed reference, pu

  _iq                spd_max;                      //!< the maximum speed, pu

  uint_least16_t     counter_current;              //!< the isr counter

  uint_least16_t     counter_isr;                  //!< the isr counter

  uint_least16_t     counter_speed;                //!< the speed counter

  uint_least16_t     counter_traj;                 //!< the traj counter

  bool             flag_enableCtrl;                //!< a flag to enable the controller
  bool             flag_enableDcBusComp;           //!< a flag to enable the DC bus compensation in the controller
  bool             flag_enablePowerWarp;           //!< a flag to enable PowerWarp
  bool             flag_enableOffset;              //!< a flag to enable offset estimation after idle state
  bool             flag_enableSpeedCtrl;           //!< a flag to enable the speed controller
  bool             flag_enableUserMotorParams;     //!< a flag to use known motor parameters from user.h file

  // NOTE:  APPENDING ONLY WORKS BECAUSE WE HAVE ALLOCATED TWO CONTROLLERS IN PROTECTED RAM AND WE ARE ONLY USING THE FIRST ONE
  MATH_vec2          Idq_offset_pu;                //!< the Idq offset values, pu
  MATH_vec2          Vdq_offset_pu;                //!< the Vdq offset values, pu
  _iq                angle_pu;                     //!< the angle value, pu
  _iq                speed_ref_pu;                 //!< the speed reference, pu
  _iq                speed_fb_pu;                  //!< the feedback speed value, pu
  _iq                speed_outMax_pu;              //!< the maximum output of the speed PI control, pu
  bool               flag_enableCurrentCtrl;       //!< a flag to enable the current controllers
} CTRL_Obj;


//! \brief Defines the CTRL handle
//!
typedef struct _CTRL_Obj_ *CTRL_Handle;


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _CTRL_OBJ_H_ definition

