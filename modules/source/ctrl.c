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

 
 
//! \file   solutions/instaspin_foc/src/ctrl.c
//! \brief  Contains the various functions related to the controller (CTRL) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>


// drivers


// modules
#include "dlog4ch.h"
#include "math.h"


// platforms
#include "ctrl.h"
#include "hal.h"
#include "user.h"


#ifdef FLASH
#pragma CODE_SECTION(CTRL_run,"ramfuncs");
#pragma CODE_SECTION(CTRL_setup,"ramfuncs");
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

void CTRL_getGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   _iq *pKp,_iq *pKi,_iq *pKd)
{

  *pKp = CTRL_getKp(handle,ctrlType);
  *pKi = CTRL_getKi(handle,ctrlType);
  *pKd = CTRL_getKd(handle,ctrlType);

  return;    
} // end of CTRL_getGains() function


void CTRL_getIab_filt_pu(CTRL_Handle handle,MATH_vec2 *pIab_filt_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIab_filt_pu->value[0] = obj->Iab_filt.value[0];
  pIab_filt_pu->value[1] = obj->Iab_filt.value[1];

  return;
} // end of CTRL_getIab_filt_pu() function


void CTRL_getIab_in_pu(CTRL_Handle handle,MATH_vec2 *pIab_in_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIab_in_pu->value[0] = obj->Iab_in.value[0];
  pIab_in_pu->value[1] = obj->Iab_in.value[1];

  return;
} // end of CTRL_getIab_in_pu() function


void CTRL_getIdq_in_pu(CTRL_Handle handle,MATH_vec2 *pIdq_in_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_in_pu->value[0] = obj->Idq_in.value[0];
  pIdq_in_pu->value[1] = obj->Idq_in.value[1];

  return;
} // end of CTRL_getIdq_in_pu() function


void CTRL_getIdq_ref_pu(CTRL_Handle handle,MATH_vec2 *pIdq_ref_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_ref_pu->value[0] = obj->Idq_ref.value[0];
  pIdq_ref_pu->value[1] = obj->Idq_ref.value[1];

  return;
} // end of CTRL_getIdq_ref_pu() function


_iq CTRL_getMagCurrent_pu(CTRL_Handle handle)
{

  return(CTRL_getIdRated_pu(handle));
} // end of CTRL_getMagCurrent_pu() function


_iq CTRL_getMaximumSpeed_pu(CTRL_Handle handle)
{

  return(CTRL_getSpd_max_pu(handle));
} // end of CTRL_getMaximumSpeed_pu() function


void CTRL_getVab_in_pu(CTRL_Handle handle,MATH_vec2 *pVab_in_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVab_in_pu->value[0] = obj->Vab_in.value[0];
  pVab_in_pu->value[1] = obj->Vab_in.value[1];

  return;
} // end of CTRL_getVab_in_pu() function


void CTRL_getVab_out_pu(CTRL_Handle handle,MATH_vec2 *pVab_out_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVab_out_pu->value[0] = obj->Vab_out.value[0];
  pVab_out_pu->value[1] = obj->Vab_out.value[1];

  return;
} // end of CTRL_getVab_out_pu() function


void CTRL_getVdq_out_pu(CTRL_Handle handle,MATH_vec2 *pVdq_out_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVdq_out_pu->value[0] = obj->Vdq_out.value[0];
  pVdq_out_pu->value[1] = obj->Vdq_out.value[1];

  return;
} // end of CTRL_getVdq_out_pu() function


void CTRL_getWaitTimes(CTRL_Handle handle,uint_least32_t *pWaitTimes)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint_least16_t stateCnt;

  for(stateCnt=0;stateCnt<CTRL_numStates;stateCnt++)
    {
      pWaitTimes[stateCnt] = obj->waitTimes[stateCnt];
    }

  return;
} // end of CTRL_getWaitTimes() function


void CTRL_run(CTRL_Handle handle,HAL_Handle halHandle,
              const HAL_AdcData_t *pAdcData,
              HAL_PwmData_t *pPwmData)
{
  uint_least16_t count_isr = CTRL_getCount_isr(handle);
  uint_least16_t numIsrTicksPerCtrlTick = CTRL_getNumIsrTicksPerCtrlTick(handle);


  // if needed, run the controller
  if(count_isr >= numIsrTicksPerCtrlTick)
    {
      CTRL_State_e ctrlState = CTRL_getState(handle);

      // reset the isr count
      CTRL_resetCounter_isr(handle);

      // increment the state counter
      CTRL_incrCounter_state(handle);

      // increment the trajectory count
      CTRL_incrCounter_traj(handle);

      // run the appropriate controller
      if(ctrlState == CTRL_State_OnLine)
        {
    	  CTRL_Obj *obj = (CTRL_Obj *)handle;

          // increment the current count
          CTRL_incrCounter_current(handle);

          // increment the speed count
          CTRL_incrCounter_speed(handle);

          if(EST_getState(obj->estHandle) >= EST_State_MotorIdentified)
            {
              // run the online controller
              CTRL_runOnLine_User(handle,pAdcData,pPwmData);
            }
          else
            {
              // run the online controller
              CTRL_runOnLine(handle,pAdcData,pPwmData);
            }
        }
      else if(ctrlState == CTRL_State_OffLine)
        {
          // run the offline controller
          CTRL_runOffLine(handle,halHandle,pAdcData,pPwmData);
        }
      else if(ctrlState == CTRL_State_Idle)
        {
          // set all pwm outputs to zero
          pPwmData->Tabc.value[0] = _IQ(0.0);
          pPwmData->Tabc.value[1] = _IQ(0.0);
          pPwmData->Tabc.value[2] = _IQ(0.0);
        }
    }
  else
    {
      // increment the isr count
      CTRL_incrCounter_isr(handle);
    }

  return;
} // end of CTRL_run() function


void CTRL_setGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   const _iq Kp,const _iq Ki,const _iq Kd)
{

  CTRL_setKp(handle,ctrlType,Kp);
  CTRL_setKi(handle,ctrlType,Ki);
  CTRL_setKd(handle,ctrlType,Kd);

  return;    
} // end of CTRL_setGains() function


void CTRL_setMagCurrent_pu(CTRL_Handle handle,const _iq magCurrent_pu)
{

  CTRL_setIdRated_pu(handle,magCurrent_pu);

  return;    
} // end of CTRL_setMagCurrent_pu() function


void CTRL_setMaximumSpeed_pu(CTRL_Handle handle,const _iq maxSpeed_pu)
{

  CTRL_setSpd_max_pu(handle,maxSpeed_pu);

  return;    
} // end of CTRL_setMaximumSpeed_pu() function


void CTRL_setParams(CTRL_Handle handle,USER_Params *pUserParams)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  _iq Kp,Ki,Kd;
  _iq outMin,outMax;
  _iq maxModulation;

  MATH_vec2 Iab_out_pu = {_IQ(0.0),_IQ(0.0)};
  MATH_vec2 Idq_out_pu = {_IQ(0.0),_IQ(0.0)};
  MATH_vec2 Idq_ref_pu = {_IQ(0.0),_IQ(0.0)};
  MATH_vec2 Vab_in_pu = {_IQ(0.0),_IQ(0.0)};
  MATH_vec2 Vab_out_pu = {_IQ(0.0),_IQ(0.0)};
  MATH_vec2 Vdq_out_pu = {_IQ(0.0),_IQ(0.0)};


  // assign the motor type
  CTRL_setMotorParams(handle,pUserParams->motor_type,
                      pUserParams->motor_numPolePairs,
                      pUserParams->motor_ratedFlux,
                      pUserParams->motor_Ls_d,
                      pUserParams->motor_Ls_q,
                      pUserParams->motor_Rr,
                      pUserParams->motor_Rs);


  // assign other controller parameters
  CTRL_setNumIsrTicksPerCtrlTick(handle,pUserParams->numIsrTicksPerCtrlTick);
  CTRL_setNumCtrlTicksPerCurrentTick(handle,pUserParams->numCtrlTicksPerCurrentTick);
  CTRL_setNumCtrlTicksPerSpeedTick(handle,pUserParams->numCtrlTicksPerSpeedTick);
  CTRL_setNumCtrlTicksPerTrajTick(handle,pUserParams->numCtrlTicksPerTrajTick);

  CTRL_setCtrlFreq_Hz(handle,pUserParams->ctrlFreq_Hz);
  CTRL_setTrajFreq_Hz(handle,pUserParams->trajFreq_Hz);
  CTRL_setTrajPeriod_sec(handle,_IQ(1.0/pUserParams->trajFreq_Hz));

  CTRL_setCtrlPeriod_sec(handle,pUserParams->ctrlPeriod_sec);

  CTRL_setMaxVsMag_pu(handle,_IQ(pUserParams->maxVsMag_pu));

  CTRL_setIab_in_pu(handle,&Iab_out_pu);
  CTRL_setIdq_in_pu(handle,&Idq_out_pu);
  CTRL_setIdq_ref_pu(handle,&Idq_ref_pu);

  CTRL_setIdRated_pu(handle,_IQ(pUserParams->IdRated/pUserParams->iqFullScaleCurrent_A));

  CTRL_setVab_in_pu(handle,&Vab_in_pu);
  CTRL_setVab_out_pu(handle,&Vab_out_pu);
  CTRL_setVdq_out_pu(handle,&Vdq_out_pu);

  CTRL_setSpd_out_pu(handle,_IQ(0.0));

  CTRL_setRhf(handle,0.0);
  CTRL_setLhf(handle,0.0);
  CTRL_setRoverL(handle,0.0);


  // reset the counters
  CTRL_resetCounter_current(handle);
  CTRL_resetCounter_isr(handle);
  CTRL_resetCounter_speed(handle);
  CTRL_resetCounter_state(handle);
  CTRL_resetCounter_traj(handle);


  // set the wait times for each state
  CTRL_setWaitTimes(handle,&pUserParams->ctrlWaitTime[0]);


  // set flags
  CTRL_setFlag_enablePowerWarp(handle,false);
  CTRL_setFlag_enableCtrl(handle,false);
  CTRL_setFlag_enableOffset(handle,true);
  CTRL_setFlag_enableSpeedCtrl(handle,true);
  CTRL_setFlag_enableUserMotorParams(handle,false);
  CTRL_setFlag_enableDcBusComp(handle,true);


  // initialize the controller error code
  CTRL_setErrorCode(handle,CTRL_ErrorCode_NoError);


  // set the default controller state
  CTRL_setState(handle,CTRL_State_Idle);


  // set the number of current sensors
  CTRL_setupClarke_I(handle,pUserParams->numCurrentSensors);


  // set the number of voltage sensors
  CTRL_setupClarke_V(handle,pUserParams->numVoltageSensors);


  // set the default Id PID controller parameters
  Kp = _IQ(0.1);
  Ki = _IQ(pUserParams->ctrlPeriod_sec/0.004);
  Kd = _IQ(0.0);
  outMin = _IQ(-0.95);
  outMax = _IQ(0.95);

  PID_setGains(obj->pidHandle_Id,Kp,Ki,Kd);
  PID_setUi(obj->pidHandle_Id,_IQ(0.0));
  PID_setMinMax(obj->pidHandle_Id,outMin,outMax);
  CTRL_setGains(handle,CTRL_Type_PID_Id,Kp,Ki,Kd);


  // set the default the Iq PID controller parameters
  Kp = _IQ(0.1);
  Ki = _IQ(pUserParams->ctrlPeriod_sec/0.004);
  Kd = _IQ(0.0);
  outMin = _IQ(-0.95);
  outMax = _IQ(0.95);

  PID_setGains(obj->pidHandle_Iq,Kp,Ki,Kd);
  PID_setUi(obj->pidHandle_Iq,_IQ(0.0));
  PID_setMinMax(obj->pidHandle_Iq,outMin,outMax);
  CTRL_setGains(handle,CTRL_Type_PID_Iq,Kp,Ki,Kd);


  // set the default speed PID controller parameters
  Kp = _IQ(0.02*pUserParams->maxCurrent*pUserParams->iqFullScaleFreq_Hz/pUserParams->iqFullScaleCurrent_A);
  Ki = _IQ(2.0*pUserParams->maxCurrent*pUserParams->iqFullScaleFreq_Hz*pUserParams->ctrlPeriod_sec/pUserParams->iqFullScaleCurrent_A);
  Kd = _IQ(0.0);
  outMin = _IQ(-1.0);
  outMax = _IQ(1.0);

  PID_setGains(obj->pidHandle_spd,Kp,Ki,Kd);
  PID_setUi(obj->pidHandle_spd,_IQ(0.0));
  PID_setMinMax(obj->pidHandle_spd,outMin,outMax);
  CTRL_setGains(handle,CTRL_Type_PID_spd,Kp,Ki,Kd);


  // set the speed reference
  CTRL_setSpd_ref_pu(handle,_IQ(0.0));


  // set the default Id current trajectory module parameters
  TRAJ_setIntValue(obj->trajHandle_Id,_IQ(0.0));
  TRAJ_setTargetValue(obj->trajHandle_Id,_IQ(0.0));
  TRAJ_setMinValue(obj->trajHandle_Id,_IQ(0.0));
  TRAJ_setMaxValue(obj->trajHandle_Id,_IQ(0.0));
  TRAJ_setMaxDelta(obj->trajHandle_Id,_IQ(0.0));


  // set the default the speed trajectory module parameters
  TRAJ_setIntValue(obj->trajHandle_spd,_IQ(0.0));
  TRAJ_setTargetValue(obj->trajHandle_spd,_IQ(0.0));
  TRAJ_setMinValue(obj->trajHandle_spd,_IQ(0.0));
  TRAJ_setMaxValue(obj->trajHandle_spd,_IQ(0.0));
  TRAJ_setMaxDelta(obj->trajHandle_spd,_IQ(0.0));


  // set the default maximum speed trajectory module parameters
  TRAJ_setIntValue(obj->trajHandle_spdMax,_IQ(0.0));
  TRAJ_setTargetValue(obj->trajHandle_spdMax,_IQ(0.0));
  TRAJ_setMinValue(obj->trajHandle_spdMax,_IQ(0.0)); // not used
  TRAJ_setMaxValue(obj->trajHandle_spdMax,_IQ(0.0)); // not used
  TRAJ_setMaxDelta(obj->trajHandle_spdMax,_IQ(0.0)); // not used

  
  // set the default estimator parameters
  CTRL_setEstParams(obj->estHandle,pUserParams);


  // set the maximum modulation for the SVGEN module
  maxModulation = _IQ(MATH_TWO_OVER_THREE);
  SVGEN_setMaxModulation(obj->svgenHandle,maxModulation);

  return;
} // end of CTRL_setParams() function


void CTRL_setSpd_ref_pu(CTRL_Handle handle,const _iq spd_ref_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->spd_ref = spd_ref_pu;

  return;
} // end of CTRL_setSpd_ref_pu() function


void CTRL_setSpd_ref_krpm(CTRL_Handle handle,const _iq spd_ref_krpm)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  _iq krpm_to_pu_sf = EST_get_krpm_to_pu_sf(obj->estHandle);

  _iq spd_ref_pu = _IQmpy(spd_ref_krpm,krpm_to_pu_sf);

  obj->spd_ref = spd_ref_pu;

  return;
} // end of CTRL_setSpd_ref_krpm() function


void CTRL_setup(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least16_t count_traj = CTRL_getCount_traj(handle);
  uint_least16_t numCtrlTicksPerTrajTick = CTRL_getNumCtrlTicksPerTrajTick(handle);


  // as needed, update the trajectory
  if(count_traj >= numCtrlTicksPerTrajTick)
    {
      _iq intValue_Id = TRAJ_getIntValue(obj->trajHandle_Id);

      // reset the trajectory count
      CTRL_resetCounter_traj(handle);

      // run the trajectories
      CTRL_runTraj(handle);
    } // end of if(gFlag_traj) block

  return;
} // end of CTRL_setup() function


void CTRL_setupClarke_I(CTRL_Handle handle,uint_least8_t numCurrentSensors)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  _iq alpha_sf,beta_sf;
  

  // initialize the Clarke transform module for current
  if(numCurrentSensors == 3)
    {
      alpha_sf = _IQ(MATH_ONE_OVER_THREE);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
  else if(numCurrentSensors == 2)
    {
      alpha_sf = _IQ(1.0);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
  else 
    {
      alpha_sf = _IQ(0.0);
      beta_sf = _IQ(0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(obj->clarkeHandle_I,alpha_sf,beta_sf);
  CLARKE_setNumSensors(obj->clarkeHandle_I,numCurrentSensors);

  return;
} // end of CTRL_setupClarke_I() function


void CTRL_setupClarke_V(CTRL_Handle handle,uint_least8_t numVoltageSensors)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  _iq alpha_sf,beta_sf;
  

  // initialize the Clarke transform module for current
  if(numVoltageSensors == 3)
    {
      alpha_sf = _IQ(MATH_ONE_OVER_THREE);
      beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
 else 
    {
      alpha_sf = _IQ(0.0);
      beta_sf = _IQ(0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(obj->clarkeHandle_V,alpha_sf,beta_sf);
  CLARKE_setNumSensors(obj->clarkeHandle_V,numVoltageSensors);

  return;
} // end of CTRL_setupClarke_V() function


void CTRL_setup_user(CTRL_Handle handle,
                     const _iq angle_pu,
                     const _iq speed_ref_pu,
                     const _iq speed_fb_pu,
                     const _iq speed_outMax_pu,
                     const MATH_vec2 *pIdq_offset_pu,
                     const MATH_vec2 *pVdq_offset_pu,
                     const bool flag_enableSpeedCtrl,
                     const bool flag_enableCurrentCtrl)
{
  CTRL_State_e ctrlState = CTRL_getState(handle);
  uint_least16_t count_traj = CTRL_getCount_traj(handle);
  uint_least16_t numCtrlTicksPerTrajTick = CTRL_getNumCtrlTicksPerTrajTick(handle);


  // increment the state counter
  CTRL_incrCounter_state(handle);

  // increment the trajectory count
  CTRL_incrCounter_traj(handle);

  // run the appropriate controller
  if(ctrlState == CTRL_State_OnLine)
  {
    // increment the current count
    CTRL_incrCounter_current(handle);

    // increment the speed count
    CTRL_incrCounter_speed(handle);
  }

  // as needed, update the trajectory
  if(count_traj >= numCtrlTicksPerTrajTick)
  {
    // reset the trajectory count
    CTRL_resetCounter_traj(handle);

    // run the trajectories
    CTRL_runTraj(handle);
  } // end of if(gFlag_traj) block


  CTRL_setAngle_pu(handle,angle_pu);
  CTRL_setSpeed_ref_pu(handle,speed_ref_pu);
  CTRL_setSpeed_fb_pu(handle,speed_fb_pu);
  CTRL_setSpeed_outMax_pu(handle,speed_outMax_pu);

  CTRL_setIdq_offset_pu(handle,pIdq_offset_pu);
  CTRL_setVdq_offset_pu(handle,pVdq_offset_pu);

  CTRL_setFlag_enableSpeedCtrl(handle,flag_enableSpeedCtrl);
  CTRL_setFlag_enableCurrentCtrl(handle,flag_enableCurrentCtrl);


  return;
} // end of CTRL_setup_user() function


void CTRL_setWaitTimes(CTRL_Handle handle,const uint_least32_t *pWaitTimes)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint_least16_t stateCnt;

  for(stateCnt=0;stateCnt<CTRL_numStates;stateCnt++)
    {
      obj->waitTimes[stateCnt] = pWaitTimes[stateCnt];
    }

  return;
} // end of CTRL_setWaitTimes() function


bool CTRL_updateState(CTRL_Handle handle)
{
  CTRL_State_e ctrlState = CTRL_getState(handle);
  bool flag_enableCtrl = CTRL_getFlag_enableCtrl(handle);
  bool stateChanged = false;


  if(flag_enableCtrl)
    {
      uint_least32_t waitTime = CTRL_getWaitTime(handle,ctrlState);
      uint_least32_t counter_ctrlState = CTRL_getCount_state(handle);


      // check for errors
      CTRL_checkForErrors(handle);


      if(counter_ctrlState >= waitTime)
        {
          // reset the counter
          CTRL_resetCounter_state(handle);


          if(ctrlState == CTRL_State_OnLine)
            {
              CTRL_Obj *obj = (CTRL_Obj *)handle;
              _iq Id_target = TRAJ_getTargetValue(obj->trajHandle_Id);

              // update the estimator state
              bool flag_estStateChanged = EST_updateState(obj->estHandle,Id_target);

              if(flag_estStateChanged)
                {
                  // setup the controller
                  CTRL_setupCtrl(handle);

                  // setup the trajectory
                  CTRL_setupTraj(handle);
                }

              if(EST_isOnLine(obj->estHandle))
                {
                  // setup the estimator for online state
                  CTRL_setupEstOnLineState(handle);
                }

              if(EST_isLockRotor(obj->estHandle) || 
                 (EST_isIdle(obj->estHandle) && EST_isMotorIdentified(obj->estHandle)))
                {
                  // set the enable controller flag to false
                  CTRL_setFlag_enableCtrl(handle,false);

                  // set the next controller state
                  CTRL_setState(handle,CTRL_State_Idle);
                }
            }
          else if(ctrlState == CTRL_State_OffLine)
            {
              // set the next controller state
              CTRL_setState(handle,CTRL_State_OnLine);
            }
          else if(ctrlState == CTRL_State_Idle)
            {
              CTRL_Obj *obj = (CTRL_Obj *)handle;
              bool  flag_enableUserMotorParams = CTRL_getFlag_enableUserMotorParams(handle);

              if(flag_enableUserMotorParams)
                {
                  // initialize the motor parameters using values from the user.h file
                  CTRL_setUserMotorParams(handle);
                }

              if(EST_isIdle(obj->estHandle))
                {
                  // setup the estimator for idle state
                  CTRL_setupEstIdleState(handle);

                  if(EST_isMotorIdentified(obj->estHandle))
                    {
                      if(CTRL_getFlag_enableOffset(handle))
                        {
                          // set the next controller state
                          CTRL_setState(handle,CTRL_State_OffLine);
                        }
                      else
                        {
                          // set the next controller state
                          CTRL_setState(handle,CTRL_State_OnLine);
                        }
                    }
                  else
                    {
                      // set the next controller state
                      CTRL_setState(handle,CTRL_State_OffLine);
                    }
                }
              else if(EST_isLockRotor(obj->estHandle))
                {
                  // set the next controller state
                  CTRL_setState(handle,CTRL_State_OnLine);
                }
            }
        }  // if(counter_ctrlState >= waitTime) loop
    } 
  else
    {
      CTRL_Obj *obj = (CTRL_Obj *)handle;

      // set the next controller state
      CTRL_setState(handle,CTRL_State_Idle);

      // set the estimator to idle
      if(!EST_isLockRotor(obj->estHandle))
        {
          if(EST_isMotorIdentified(obj->estHandle))
            {
              EST_setIdle(obj->estHandle);
            }
          else
            {
              EST_setIdle_all(obj->estHandle);

              EST_setRs_pu(obj->estHandle,_IQ30(0.0));
            }
        }
    }


  // check to see if the state changed
  if(ctrlState != CTRL_getState(handle))
    {
      stateChanged = true;
    }

  return(stateChanged);
} // end of CTRL_updateState() function

// end of file
