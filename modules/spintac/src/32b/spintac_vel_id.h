#ifndef __SPINTAC_VEL_ID_H__
#define __SPINTAC_VEL_ID_H__
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
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
 * *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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

//! \file    modules/spintac/src/32b/spintac_vel_id.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Velocity Identify component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACVELID SpinTAC Velocity ID
//@{

#include "spintac_version.h"

//! \brief Enumeration for the Velocity Identify Status states
//!
typedef enum
{
  ST_VEL_ID_IDLE=0,	//!< Velocity Identify is in idle state, zero output
  ST_VEL_ID_INIT,	//!< Velocity Identify is in init state, validating configured parameters
  ST_VEL_ID_BUSY	//!< Velocity Identify is in busy state, identifying system inertia
} ST_VelIdStatus_e;

//! \brief      Defines the ST_VelIdCfg_t data
//! \details    The ST_VelIdCfg_t object contains all configuration parameters
//!				of the Velocity ID object.
typedef struct {
  // Configuration parameters: Can be set only once at system startup
  _iq24  T_sec;			//!< Sample time 				{ unit: [s], value range: (0.0, 0.01] }
  int16_t LpfTime_tick;	//!< Low pass filter ISR ticks 	{ unit: [ticks], value range: [1, 100] }
  _iq24  TimeOut_sec;	//!< Maximum time allowed for inertia estimation process	{ unit: [s], value range: [1.0, 10.0] }

  // Config parameters that may be modified
  _iq24  GoalSpeed;		//!< Speed to reach in inertia estimation	{ unit: [pu/s], value range: (0, 1.0] }
  _iq24  OutMax;	    //!< Control signal limit	{ unit: [PU], value range: (0, 1.0] }
  _iq24  RampTime_sec; 	//!< Torque acceleration ramp time { unit [s], value range: [T_sec, 25.0] }
  bool Sensorless;      //!< Indicates if the FOC is using a sensorless estimator
} ST_VelIdCfg_t;	 	// Structure for SpinTAC Identify configuration


//! \brief      Defines the ST_VelId_t data
//! \details    The ST_VelId_t object contains all parameters needed to
//!				perform Velocity ID
typedef struct {
  // Configuration structure
  ST_VelIdCfg_t cfg;		//!< Configuration substructure
  /* Input Variables */
  _iq24  VelFdb;			//!< Velocity feedback { unit: [pu/s], value range: [-1.0, 1.0] }
  // Control bits
  bool ENB;				    //!< Enable bit { false: enable; true: disable }
  /* Output Variables */
  _iq24  Out;				//!< control output { unit: [PU], value range: [-cfg.OutMax, cfg.OutMax] }
  // Estimation Result
  _iq24  InertiaEst;		//!< Estimated Inertia { unit: [PU/(pu/s^2)], value range: positive _IQ24 value }
  _iq24  FrictionEst;		//!< Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
  // Information variables
  ST_VelIdStatus_e STATUS;	//!< Status { ST_VEL_ID_IDLE, ST_VEL_ID_INIT, ST_VEL_ID_BUSY }
  uint16_t ERR_ID;			//!< Error ID { 0: no error; others: see error code }
  /* Internal variables */
  uint32_t s0[9];
} ST_VelId_t;	// Structure for SpinTAC Identify

typedef struct _ST_VELID_Handle_ *ST_VELID_Handle; // SpinTAC Velocity Identify Handle

//! \brief      Sets the Sample Time (cfg.T_sec) for SpinTAC Velocity Identify
//! \param[in]  handle     The handle for the SpinTAC Velocity Identify Object
//! \param[in]  sampleTime Sample Time { unit: [s], value range: (0, 0.01] }
static inline void STVELID_setSampleTime_sec(ST_VELID_Handle handle, _iq24 sampleTime) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	if(obj->STATUS == ST_VEL_ID_IDLE) {
		obj->cfg.T_sec = sampleTime;
	}

	return;
} // end of STVELID_setSampleTime_sec function

//! \brief      Sets the Maximum Output (cfg.OutMax) for SpinTAC Velocity Identify
//! \param[in]  handle The handle for the SpinTAC Velocity Identify Object
//! \param[in]  outMax Control signal high limit { unit: [PU], Value range: (0.0, 1.0] }
static inline void STVELID_setOutputMaximum(ST_VELID_Handle handle, _iq24 outMax) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	if(obj->STATUS == ST_VEL_ID_IDLE) {
		obj->cfg.OutMax = outMax;
	}

	return;
} // end of STVELID_setOutputMaximum function

//! \brief      Sets the Goal Speed of inertia identification (cfg.GoalSpeed) for SpinTAC Velocity Identify
//! \param[in]  handle    The handle for the SpinTAC Velocity Identify Object
//! \param[in]  goalSpeed Inertia identification goal speed	{ unit: [pu/s], Value range: (0.0, 1.0] }
static inline void STVELID_setGoalSpeed(ST_VELID_Handle handle, _iq24 goalSpeed) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	obj->cfg.GoalSpeed = goalSpeed;

	return;
} // end of STVELID_setGoalSpeed function

//! \brief      Gets the Goal Speed of inertia identification (cfg.GoalSpeed) for SpinTAC Velocity Identify
//! \param[in]  handle          The handle for the SpinTAC Velocity Identify Object
//! \return     _iq24 GoalSpeed Inertia identification goal speed	{ unit: [pu/s], Value range: (0.0, 1.0] }
static inline _iq24 STVELID_getGoalSpeed(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return(obj->cfg.GoalSpeed);
} // end of STVELID_getGoalSpeed function

//! \brief      Sets the Low Pass Filter Time Constant (cfg.LpfTime_tick) for SpinTAC Velocity Identify
//! \param[in]  handle  The handle for the SpinTAC Velocity Identify Object
//! \param[in]  lpfTime Low pass filter ISR ticks 	{ unit: [ticks], value range: [1, 100] }
static inline void STVELID_setLowPassFilterTime_tick(ST_VELID_Handle handle, int16_t lpfTime) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	if(obj->STATUS == ST_VEL_ID_IDLE) {
		obj->cfg.LpfTime_tick = lpfTime;
	}

	return;
} // end of STVELID_setLowPassFilterTime_tick function

//! \brief      Sets the Time Out Time (cfg.TimeOut_sec) for SpinTAC Velocity Identify
//! \param[in]  handle  The handle for the SpinTAC Velocity Identify Object
//! \param[in]  timeOut Maximum time allowed for inertia estimation process { unit: [s], Value range: [1.0, 10.0] }
static inline void STVELID_setTimeOut_sec(ST_VELID_Handle handle, _iq24 timeOut) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	if(obj->STATUS == ST_VEL_ID_IDLE) {
		obj->cfg.TimeOut_sec = timeOut;
	}

	return;
} // end of STVELID_setTimeOut_sec function

//! \brief      Sets the Torque Ramp Time (cfg.RampTime_sec) for SpinTAC Velocity Identify
//! \param[in]  handle   The handle for the SpinTAC Velocity Identify Object
//! \param[in]  rampTime Torque acceleration ramp time { unit [s], Value range: [cfg.T_sec, 25.0] }
static inline void STVELID_setTorqueRampTime_sec(ST_VELID_Handle handle, _iq24 rampTime) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	if(obj->STATUS == ST_VEL_ID_IDLE) {
		obj->cfg.RampTime_sec = rampTime;
	}

	return;
} // end of STVELID_setTorqueRampTime_sec function

//! \brief      Gets the Torque Ramp Time (cfg.RampTime_sec) for SpinTAC Velocity Identify
//! \param[in]  handle             The handle for the SpinTAC Velocity Identify Object
//! \return     _iq24 RampTime_sec Torque acceleration ramp time { unit [s], Value range: [cfg.T_sec, 25.0] }
static inline _iq24 STVELID_getTorqueRampTime_sec(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return(obj->cfg.RampTime_sec);
} // end of STVELID_getTorqueRampTime_sec function

//! \brief      Sets the feedback type (cfg.Sensorless) for SpinTAC Velocity Identify
//! \param[in]  handle     The handle for the SpinTAC Velocity Identify Object
//! \param[in]  sensorless Indicates if the FOC is using a sensorless estimator { false: sensor; true: sensorless }
static inline void STVELID_setSensorlessFeedback(ST_VELID_Handle handle, bool sensorless) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	if(obj->STATUS == ST_VEL_ID_IDLE) {
		obj->cfg.Sensorless = sensorless;
	}

	return;
} // end of STVELID_setTorqueRampTime_sec function

//! \brief      Sets the Velocity Feedback (VelFdb) for SpinTAC Velocity Identify
//! \param[in]  handle The handle for the SpinTAC Velocity Identify Object
//! \param[in]  velFdb Velocity feedback  { unit [pu/s], value range: [-1.0, 1.0] }
static inline void STVELID_setVelocityFeedback(ST_VELID_Handle handle, _iq24 velFdb) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	obj->VelFdb = velFdb;

	return;
} // end of STVELID_setVelocityFeedback function

//! \brief      Gets the Torque (Iq) Reference value (Out) from SpinTAC Velocity Identify
//! \param[in]  handle    The handle for the SpinTAC Velocity Identify Object
//! \return     _iq24 Out Torque (Iq) Reference  { unit [PU], value range: [-cfg.OutMax, cfg.OutMax] }
static inline _iq24 STVELID_getTorqueReference(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return (obj->Out);
} // end of STVELID_getTorqueReference function

//! \brief      Sets the Torque (Iq) Reference (Out) for SpinTAC Velocity Identify
//! \param[in]  handle The handle for the SpinTAC Velocity Identify Object
//! \param[in]  out    Torque (Iq) Reference  { unit [PU], value range: [-cfg.OutMax, cfg.OutMax] }
static inline void STVELID_setTorqueReference(ST_VELID_Handle handle, _iq24 out) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	obj->Out = out;

	return;
} // end of STVELID_setTorqueReference function

//! \brief      Gets the System Inertia (InertiaEst) from SpinTAC Velocity Identify
//! \param[in]  handle           The handle for the SpinTAC Velocity Identify Object
//! \return     _iq24 InertiaEst Estimated Inertia { unit: [PU/(pu/s^2)], value range: positive _IQ24 value }
static inline _iq24 STVELID_getInertiaEstimate(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return (obj->InertiaEst);
} // end of STVELID_getInertiaEstimate function

//! \brief      Gets the System Friction (FrictionEst) from SpinTAC Velocity Identify
//! \param[in]  handle            The handle for the SpinTAC Velocity Identify Object
//! \return     _iq24 FrictionEst Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
static inline _iq24 STVELID_getFrictionEstimate(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return (obj->FrictionEst);
} // end of STVELID_getFrictionEstimate function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Velocity Identify
//! \param[in]  handle The handle for the SpinTAC Velocity Identify Object
//! \param[in]  enb    Enable bit { false: disable; true: enable }
static inline void STVELID_setEnable(ST_VELID_Handle handle, bool enb) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	obj->ENB = enb;

	return;
} // end of STVELID_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Velocity Identify
//! \param[in]  handle   The handle for the SpinTAC Velocity Identify Object
//! \return     bool ENB Enable bit { false: disable; true: enable }
static inline bool STVELID_getEnable(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return (obj->ENB);
} // end of STVELID_getEnable function

//! \brief      Gets the Status value (STATUS) for SpinTAC Velocity Identify
//! \param[in]  handle                  The handle for the SpinTAC Velocity Identify Object
//! \return     ST_VelIdStatus_e STATUS Status { ST_VEL_ID_IDLE, ST_VEL_ID_INIT, ST_VEL_ID_BUSY }
static inline ST_VelIdStatus_e STVELID_getStatus(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return (obj->STATUS);
} // end of STVELID_getStatus function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Velocity Identify
//! \param[in]  handle          The handle for the SpinTAC Velocity Identify Object
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STVELID_getErrorID(ST_VELID_Handle handle) {
	ST_VelId_t *obj = (ST_VelId_t *)handle;

	return (obj->ERR_ID);
} // end of STVELID_getErrorID function

//! \brief      Initializes the SpinTAC Velocity Identify object
//! \param[in]   *pMemory               Pointer to the memory for ST_VelId_t
//! \param[in]   numBytes               The number of bytes in the ST_VelId_t
//! \return      ST_VELID_Handle handle The handle for the SpinTAC Velocity Identify Object
ST_VELID_Handle STVELID_init(void *pMemory, const size_t numBytes);

//! \brief      Runs the SpinTAC Identify function
//! \param[in]  handle The handle for the SpinTAC Velocity Identify Object
void STVELID_run(ST_VELID_Handle handle);

//@} // defgroup
#endif //__SPINTAC_VEL_ID_H__

