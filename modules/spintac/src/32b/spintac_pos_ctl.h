#ifndef __SPINTAC_POS_CTL_H__
#define __SPINTAC_POS_CTL_H__
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

//! \file    modules/spintac/src/32b/spintac_pos_ctl.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Position Control component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACPOSCTL SpinTAC Position Control
//@{

#include "spintac_version.h"

#ifndef __ST_AXIS_ENUM__
#define __ST_AXIS_ENUM__
//! \brief Enumeration for the Axis Status states
//!
typedef enum
{
  ST_AXIS0=0,	//!< First Axis
  ST_AXIS1		//!< Second Axis
} ST_Axis_e;
#endif //__ST_AXIS_ENUM__

#ifndef __ST_CTL_STATUS_ENUM__
#define __ST_CTL_STATUS_ENUM__
//! \brief Enumeration for the Control Status states
//!
typedef enum
{
  ST_CTL_IDLE=0,	//!< ST_CTL in idle state
  ST_CTL_INIT,		//!< ST_CTL in init state
  ST_CTL_CONF,		//!< ST_CTL in conf state
  ST_CTL_BUSY		//!< ST_CTL in busy state
} ST_CtlStatus_e;
#endif //__ST_CTL_STATUS_ENUM__

//! \brief      Defines the ST_PosCtlCfg_t data
//! \details    The ST_PosCtlCfg_t object contains all configuration parameters
//!				of the Position Control component.
typedef struct {
  //System config parameters: Can be set only once at system startup
  ST_Axis_e Axis;		//!< Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
  _iq24	T_sec;			//!< Sample time { unit: [s], value range: (0, 0.01] }
  _iq24	ROMax_mrev;		//!< Position Rollover bound { unit: [MRev], value range: [2, 100] }
  _iq24	mrev_TO_pu;     //!< Conversion ratio from mechanical revolution to pu { value range: [0.002, 1]) }
  _iq24	PosErrMax_mrev;	//!< Maximum allowable position error { unit: [MRev], value range (0, ROMax_mrev/2] }
  bool RampDist;		//!< Disturbance type { false: Step Disturbance, true: Ramp Disturbance }
  bool FiltEN;		    //!< Enable low-pass Filter { false: Filter Disabled; true: Filter Enabled }

  //Config parameters that may be modified
  _iq24	OutMax;			//!< Control signal upper limit { unit: [PU], value range: (-1, 1] }
  _iq24	OutMin;			//!< Control signal lower limit { unit: [PU], value range: [-1, OutMax) }
  _iq24	VelMax;			//!< Velocity reference signal upper limit { unit: [pu/s], Value range: (0, 1] }
} ST_PosCtlCfg_t;		//!< Structure for SpinTAC Control-Position configuration

//! \brief      Defines the ST_PosCtl_t data
//! \details    The ST_PosCtl_t object contains all parameters needed to
//!				perform Position Control
typedef struct {
  /* Configuration variables */
  ST_PosCtlCfg_t cfg;
  /* Input variables */
  _iq24	PosRef_mrev;	//!< Position reference { unit: [MRev], value range: [-ROMax, ROMax] }
  _iq24	VelRef;			//!< Velocity reference { unit: [pu/s] }
  _iq24	AccRef;			//!< Acceleration reference { unit: [pu/s^2] }
  _iq24	PosFdb_mrev;   	//!< Position feedback { Unit: [MRev], value range: [-ROMax, ROMax) }
  _iq24	Inertia;		//!< System inertia { unit: [PU/(pu/s^2)], value range: positive _IQ24 value }
  _iq24 Friction;		//!< Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
  _iq20	Bw_radps;		//!< Controller Bandwidth { unit: [rad/s], value range: [0.01, 1000.0] }
  // Control bits
  bool ENB;			    //!< Enable bit { false: disable; true: enable }
  /* Output variables */
  _iq24	Out;   			//!< Control output { unit: [PU] }
  // Information variables
  ST_CtlStatus_e STATUS;//!< Status { ST_CTL_IDLE, ST_CTL_INIT, ST_CTL_CONF, ST_CTL_BUSY }
  uint16_t	ERR_ID;		//!< Error ID { 0: no error; others: see error code }
  _iq24 PosErr_mrev;	//!< Position error { unit: [MRev] }
  /* Internal variables */
  uint32_t s1[18];
} ST_PosCtl_t;	// Structure for SpinTAC Position Control

typedef struct _ST_POSCTL_Handle_ *ST_POSCTL_Handle; // SpinTAC Position Controller Handle

//! \brief      Sets the Axis (cfg.Axis) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  axis   Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
static inline void STPOSCTL_setAxis(ST_POSCTL_Handle handle, ST_Axis_e axis) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.Axis = axis;
	}

	return;
} // end of STPOSCTL_setAxis function

//! \brief      Sets the Sample Time (cfg.T_sec) for SpinTAC Position Controller
//! \param[in]  handle     The handle for the SpinTAC Position Controller Object
//! \param[in]  sampleTime Sample Time { unit: [s], value range: (0, 0.01] }
static inline void STPOSCTL_setSampleTime_sec(ST_POSCTL_Handle handle, _iq24 sampleTime) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.T_sec = sampleTime;
	}

	return;
} // end of STPOSCTL_setSampleTime_sec function

//! \brief      Gets the Maximum Output (cfg.OutMax) for SpinTAC Position Controller
//! \param[in]  handle       The handle for the SpinTAC Position Controller Object
//! \return     _iq24 OutMax Control signal upper limit { unit: [PU], value range: (-1, 1] }
static inline _iq24 STPOSCTL_getOutputMaximum(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return obj->cfg.OutMax;
} // end of STPOSCTL_getOutputMaximum function

//! \brief      Gets the Minimum Output (cfg.OutMin) for SpinTAC Position Controller
//! \param[in]  handle       The handle for the SpinTAC Position Controller Object
//! \return     _iq24 OutMin Control signal lower limit { unit: [PU], value range: [-1, OutMax) }
static inline _iq24 STPOSCTL_getOutputMinimum(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return obj->cfg.OutMin;
} // end of STPOSCTL_getOutputMinimum function


//! \brief      Sets the Maximum Output (cfg.OutMax) and Minimum Output (cfg.OutMin) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  outMax Control signal upper limit { unit: [PU], value range: (-1, 1] }
//! \param[in]  outMin Control signal lower limit { unit: [PU], value range: [-1, OutMax) }
static inline void STPOSCTL_setOutputMaximums(ST_POSCTL_Handle handle, _iq24 outMax, _iq24 outMin) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->cfg.OutMax = outMax;
	obj->cfg.OutMin = outMin;

	return;
} // end of STPOSCTL_setOutputMaximums function

//! \brief      Sets the Maximum Velocity (cfg.VelMax) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  velMax Velocity reference signal upper limit{ unit: [pu/s], Value range: (0, 1] }
static inline void STPOSCTL_setVelocityMaximum(ST_POSCTL_Handle handle, _iq24 velMax) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->cfg.VelMax = velMax;

	return;
} // end of STPOSCTL_setVelocityMaximum function

//! \brief      Sets the Position Rollover Maximum (cfg.ROMax_mrev) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  roMax  Position Rollover bound { unit: [MRev], value range: [2, 100] }
static inline void STPOSCTL_setPositionRolloverMaximum_mrev(ST_POSCTL_Handle handle, _iq24 roMax) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.ROMax_mrev = roMax;
	}

	return;
} // end of STPOSCTL_setPositionRolloverMaximum_mrev function

//! \brief      Sets the Unit Conversion (cfg.mrev_TO_pu) for SpinTAC Position Controller
//! \param[in]  handle    The handle for the SpinTAC Position Controller Object
//! \param[in]  baseFreq  The value that frequency is scaled with in the system { USER_IQ_FULL_SCALE_FREQ_Hz }
//! \param[in]  polePairs The number of Pole Pairs in the motor { USER_MOTOR_NUM_POLE_PAIRS }
static inline void STPOSCTL_setUnitConversion(ST_POSCTL_Handle handle, float_t baseFreq, uint16_t polePairs) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.mrev_TO_pu = _IQ24(((float_t)polePairs) / baseFreq);
	}

	return;
} // end of STPOSCTL_setUnitConversion function

//! \brief      Sets the Ramp Disturbance Flag (cfg.RampDist) for SpinTAC Position Controller
//! \param[in]  handle   The handle for the SpinTAC Position Controller Object
//! \param[in]  rampDist Disturbance type { false: Step Disturbance, true: Ramp Disturbance }
static inline void STPOSCTL_setRampDisturbanceFlag(ST_POSCTL_Handle handle, bool rampDist) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.RampDist = rampDist;
	}

	return;
} // end of STPOSCTL_setRampDisturbanceFlag function

//! \brief      Sets the Position Error Maximum (cfg.PosErrMax_mrev) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  errMax Maximum allowable position error { unit: [MRev], value range (0, ROMax/2] }
static inline void STPOSCTL_setPositionErrorMaximum_mrev(ST_POSCTL_Handle handle, _iq24 errMax) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.PosErrMax_mrev = errMax;
	}

	return;
} // end of STPOSCTL_setPositionErrorMaximum_mrev function

//! \brief      Sets the Feedback Filter Enable Flag (cfg.FiltEN) for SpinTAC Position Controller
//! \param[in]  handle       The handle for the SpinTAC Position Controller Object
//! \param[in]  filterEnable Enable low-pass Filter { true: Filter Enabled; false: Filter Disabled }
static inline void STPOSCTL_setFilterEnableFlag(ST_POSCTL_Handle handle, bool filterEnable) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.FiltEN = filterEnable;
	}

	return;
} // end of STPOSCTL_setFilterEnableFlag function

//! \brief      Sets the Position Reference (PosRef_mrev) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  posRef Position reference { unit: [MRev], value range: [-ROMax, ROMax] }
static inline void STPOSCTL_setPositionReference_mrev(ST_POSCTL_Handle handle, _iq24 posRef) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->PosRef_mrev = posRef;

	return;
} // end of STPOSCTL_setPositionReference_mrev function

//! \brief      Sets the Velocity Reference (VelRef) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  velRef Velocity reference { unit: [pu/s], value range: [-1, 1] }
static inline void STPOSCTL_setVelocityReference(ST_POSCTL_Handle handle, _iq24 velRef) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->VelRef = velRef;

	return;
} // end of STPOSCTL_setVelocityReference function

//! \brief      Sets the Acceleration Reference (AccRef) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  accRef Acceleration reference { unit: [pu/s^2], _IQ24 value) }
static inline void STPOSCTL_setAccelerationReference(ST_POSCTL_Handle handle, _iq24 accRef) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->AccRef = accRef;

	return;
} // end of STPOSCTL_setAccelerationReference function

//! \brief      Sets the Position Feedback (PosFdb_mrev) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  posFdb Position feedback { Unit: [MRev], value range: [-ROMax, ROMax) }
static inline void STPOSCTL_setPositionFeedback_mrev(ST_POSCTL_Handle handle, _iq24 posFdb) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->PosFdb_mrev = posFdb;

	return;
} // end of STPOSCTL_setPositionFeedback_mrev function

//! \brief      Sets the Inertia (Inertia) for SpinTAC Position Controller
//! \param[in]  handle  The handle for the SpinTAC Position Controller Object
//! \param[in]  inertia System inertia { unit: [PU/(pu/s^2)], Value range: positive _IQ24 value }
static inline void STPOSCTL_setInertia(ST_POSCTL_Handle handle, _iq24 inertia) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->Inertia = inertia;

	return;
} // end of STPOSCTL_setInertia function

//! \brief      Gets the Inertia (Inertia) for SpinTAC Position Controller
//! \param[in]  handle        The handle for the SpinTAC Position Controller Object
//! \return     _iq24 Inertia System inertia { unit: [PU/(pu/s^2)], Value range: positive _IQ24 value }
static inline _iq24 STPOSCTL_getInertia(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return(obj->Inertia);
} // end of STPOSCTL_getInertia function

//! \brief      Sets the Friction (Friction) for SpinTAC Position Controller
//! \param[in]  handle   The handle for the SpinTAC Position Controller Object
//! \param[in]  friction Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
static inline void STPOSCTL_setFriction(ST_POSCTL_Handle handle, _iq24 friction) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->Friction = friction;

	return;
} // end of STPOSCTL_setFriction function

//! \brief      Gets the Friction (Friction) for SpinTAC Position Controller
//! \param[in]  handle         The handle for the SpinTAC Position Controller Object
//! \return     _iq24 Friction Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
static inline _iq24 STPOSCTL_getFriction(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return(obj->Friction);
} // end of STPOSCTL_getFriction function

//! \brief      Sets the Bandwidth (Bw_radps) for SpinTAC Position Controller
//! \param[in]  handle  The handle for the SpinTAC Position Controller Object
//! \param[in]  bw      Controller Bandwidth { unit: [rad/s] }
static inline void STPOSCTL_setBandwidth_radps(ST_POSCTL_Handle handle, _iq20 bw) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->Bw_radps = bw;

	return;
} // end of STPOSCTL_setBandwidth_radps function

//! \brief      Gets the Bandwidth (Bw_radps) for SpinTAC Position Controller
//! \param[in]  handle         The handle for the SpinTAC Position Controller Object
//! \return     _iq20 Bw_radps Bandwidth Scale { unit: [rad/s] }
static inline _iq20 STPOSCTL_getBandwidth_radps(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return(obj->Bw_radps);
} // end of STPOSCTL_getBandwidth_radps function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  enb    Enable bit { true: enable; false: disable }
static inline void STPOSCTL_setEnable(ST_POSCTL_Handle handle, bool enb) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->ENB = enb;

	return;
} // end of STPOSCTL_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Position Controller
//! \param[in]  handle     The handle for the SpinTAC Position Controller Object
//! \return     bool   ENB Enable bit { true: enable; false: disable }
static inline bool STPOSCTL_getEnable(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return (obj->ENB);
} // end of STPOSCTL_getEnable function

//! \brief      Sets the Torque (Iq) Reference (Out) for SpinTAC Position Controller
//! \param[in]  handle The handle for the SpinTAC Position Controller Object
//! \param[in]  out    Control output { unit: [PU], value range: [cfg.OutMin, cfg.OutMax] }
static inline void STPOSCTL_setTorqueReference(ST_POSCTL_Handle handle, _iq24 out) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	obj->Out = out;

	return;
} // end of STPOSCTL_setTorqueReference function

//! \brief      Gets the Torque (Iq) Reference (Out) for SpinTAC Position Controller
//! \param[in]  handle  The handle for the SpinTAC Position Controller Object
//! \return     _iq24 Out Control output { unit: [PU], value range: [cfg.OutMin, cfg.OutMax] }
static inline _iq24 STPOSCTL_getTorqueReference(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return (obj->Out);

} // end of STPOSCTL_getTorqueReference function

//! \brief      Gets the Status value (STATUS) for SpinTAC Position Controller
//! \param[in]  handle                The handle for the SpinTAC Position Controller Object
//! \return     ST_CtlStatus_e status Status { ST_CTL_IDLE, ST_CTL_INIT, ST_CTL_CONF, ST_CTL_BUSY }
static inline ST_CtlStatus_e STPOSCTL_getStatus(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return (obj->STATUS);

} // end of STPOSCTL_getStatus function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Position Controller
//! \param[in]  handle          The handle for the SpinTAC Position Controller Object
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STPOSCTL_getErrorID(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return (obj->ERR_ID);

} // end of STPOSCTL_getErrorID function

//! \brief      Gets the Position Error (PosErr_mrev) for SpinTAC Position Controller
//! \param[in]  handle          The handle for the SpinTAC Position Controller Object
//! \return     _iq24 PosErr_mrev Position error	{ unit: [MRev] }
static inline _iq24 STPOSCTL_getPositionError_mrev(ST_POSCTL_Handle handle) {
	ST_PosCtl_t *obj = (ST_PosCtl_t *)handle;

	return (obj->PosErr_mrev);

} // end of STPOSCTL_getPositionError function

//! \brief      Initializes the SpinTAC Position Controller object
//! \param[in]  *pMemory                Pointer to the memory for ST_PosCtl_t
//! \param[in]  numBytes                The number of bytes in the ST_PosCtl_t
//! \return     ST_POSCTL_Handle handle The handle for the SpinTAC Position Controller Object
ST_POSCTL_Handle STPOSCTL_init(void *pMemory, const size_t numBytes);

//! \brief      Runs the SpinTAC Position Control Function
//! \param[in]  handle The handle to the Position Control structure
void STPOSCTL_run(ST_POSCTL_Handle handle);
//@} // defgroup
#endif //__SPINTAC_POS_CTL_H__
