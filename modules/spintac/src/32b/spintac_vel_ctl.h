#ifndef __SPINTAC_VEL_CTL_H__
#define __SPINTAC_VEL_CTL_H__
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

//! \file    modules/spintac/src/32b/spintac_vel_ctl.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Velocity Control component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACVELCTL SpinTAC Velocity Control
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

//! \brief      Defines the ST_VelCtlCfg_t data
//! \details    The ST_VelCtlCfg_t object contains all configuration parameters
//!				of the Velocity Control component
typedef struct {
  //System config parameters: Can be set only once at system startup
  ST_Axis_e	Axis;	//!< Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
  _iq24	T_sec;		//!< Sample time { unit: [s], value range: (0.0, 0.01] }
  bool FiltEN;	    //!< Enable low-pass Filter { false: Filter Disabled; true: Filter Enabled }
  // Config parameters that may be modified
  _iq24	OutMax;		//!< Control signal upper limit { unit: [PU], value range: (-1.0, 1.0] }
  _iq24	OutMin;		//!< Control signal lower limit { unit: [PU], value range: [-1.0, OutMax) }
} ST_VelCtlCfg_t;	// Structure for SpinTAC Control-Velocity configuration

//! \brief      Defines the ST_VelCtl_t data
//! \details    The ST_VelCtl_t object contains all parameters needed to
//!				perform Velocity Control
typedef struct {
  /* Configuration variables */
  ST_VelCtlCfg_t cfg;
  /* Input variables */
  _iq24 VelRef;   			//!< Reference input { unit: [pu/s], value range: [-1.0, 1.0] }
  _iq24 AccRef;				//!< Feedforward input { unit: [pu/s^2] }
  _iq24 VelFdb;   			//!< Feedback input { unit: [pu/s], value range: [-1.0, 1.0] }
  _iq24	Inertia;			//!< System inertia { unit: [PU/(pu/s^2)], value range: positive _IQ24 value }
  _iq24 Friction;			//!< Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
  _iq20	Bw_radps;			//!< Controller Bandwidth {unit: [rad/s], value range: [0.01, min(2000.0, 0.1/T_sec)] }
  // Control bits
  bool ENB;				    //!< Enable bit { false: disable; true: enable }
  /* Output variables */
  _iq24	Out;   				//!< Control output { unit: [PU], value range: [cfg.OutMin, cfg.OutMax] }
  // Information variables
  ST_CtlStatus_e STATUS;	//!< Status { ST_CTL_IDLE, ST_CTL_INIT, ST_CTL_CONF, ST_CTL_BUSY }
  uint16_t	ERR_ID;			//!< Error ID { 0: no error; others: see error code }
  /* Internal variables */
  uint32_t s1[10];
} ST_VelCtl_t;	// Structure for SpinTAC Velocity Control

typedef struct _ST_VELCTL_Handle_ *ST_VELCTL_Handle; // SpinTAC Velocity Controller Handle

//! \brief      Sets the Axis (cfg.Axis) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  axis   Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
static inline void STVELCTL_setAxis(ST_VELCTL_Handle handle, ST_Axis_e axis) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.Axis = axis;
	}

	return;
} // end of STVELCTL_setAxis function

//! \brief      Sets the Sample Time (cfg.T_sec) for SpinTAC Velocity Controller
//! \param[in]  handle     The handle for the SpinTAC Velocity Controller Object
//! \param[in]  sampleTime Sample Time { unit: [s], value range: (0.0, 0.01] }
static inline void STVELCTL_setSampleTime_sec(ST_VELCTL_Handle handle, _iq24 sampleTime) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.T_sec = sampleTime;
	}

	return;
} // end of STVELCTL_setSampleTime_sec function

//! \brief      Gets the Maximum Output (cfg.OutMax) for SpinTAC Velocity Controller
//! \param[in]  handle       The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 OutMax Control signal upper limit { unit: [PU], value range: [-1.0, 1.0] }
static inline _iq24 STVELCTL_getOutputMaximum(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return obj->cfg.OutMax;
} // end of STVELCTL_getOutputMaximum function

//! \brief      Gets the Minimum Output (cfg.OutMin) for SpinTAC Velocity Controller
//! \param[in]  handle       The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 OutMin Control signal lower limit { unit: [PU], value range: [-1.0, 1.0] }
static inline _iq24 STVELCTL_getOutputMinimum(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return obj->cfg.OutMin;
} // end of STVELCTL_getOutputMinimum function

//! \brief      Sets the Maximum Output (cfg.OutMax) and Minimum Output (cfg.OutMin) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  outMax Control signal upper limit { unit: [PU], value range: [-1.0, 1.0] }
//! \param[in]  outMin Control signal lower limit { unit: [PU], value range: [-1.0, OutMax] }
static inline void STVELCTL_setOutputMaximums(ST_VELCTL_Handle handle, _iq24 outMax, _iq24 outMin) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->cfg.OutMax = outMax;
	obj->cfg.OutMin = outMin;

	return;
} // end of STVELCTL_setOutputMaximums function

//! \brief      Sets the Feedback Filter Enable Flag (cfg.FiltEN) for SpinTAC Velocity Controller
//! \param[in]  handle       The handle for the SpinTAC Velocity Controller Object
//! \param[in]  filterEnable Enable low-pass Filter { false: Filter Disabled; true: Filter Enabled }
static inline void STVELCTL_setFilterEnableFlag(ST_VELCTL_Handle handle, bool filterEnable) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	if(obj->STATUS == ST_CTL_IDLE) {
		obj->cfg.FiltEN = filterEnable;
	}

	return;
} // end of STVELCTL_setFilterEnableFlag function

//! \brief      Sets the Velocity Reference (VelRef) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  velRef Reference input { unit: [pu/s], value range: [-1, 1] }
static inline void STVELCTL_setVelocityReference(ST_VELCTL_Handle handle, _iq24 velRef) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->VelRef = velRef;

	return;
} // end of STVELCTL_setVelocityReference function

//! \brief      Gets the Velocity Reference (VelRef) for SpinTAC Velocity Controller
//! \param[in]  handle       The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 VelRef Reference input { unit: [pu/s], value range: [-1.0, 1.0] }
static inline _iq24 STVELCTL_getVelocityReference(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return(obj->VelRef);
} // end of STVELCTL_getVelocityReference function

//! \brief      Sets the Acceleration Reference (AccRef) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  accRef Feedforward input { unit: [pu/s^2], _IQ24 value) }
static inline void STVELCTL_setAccelerationReference(ST_VELCTL_Handle handle, _iq24 accRef) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->AccRef = accRef;

	return;
} // end of STVELCTL_setAccelerationReference function

//! \brief      Sets the Velocity Feedback (VelFdb) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  velFdb Feedback input { unit: [pu/s], value range: [-1.0, 1.0] }
static inline void STVELCTL_setVelocityFeedback(ST_VELCTL_Handle handle, _iq24 velFdb) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->VelFdb = velFdb;

	return;
} // end of STVELCTL_setVelocityFeedback function

//! \brief      Gets the Velocity Feedback (VelFdb) for SpinTAC Velocity Controller
//! \param[in]  handle       The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 VelFdb Feedback input { unit: [pu/s], value range: [-1.0, 1.0] }
static inline _iq24 STVELCTL_getVelocityFeedback(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return(obj->VelFdb);
} // end of STVELCTL_getVelocityFeedback function

//! \brief      Sets the Inertia (Inertia) for SpinTAC Velocity Controller
//! \param[in]  handle  The handle for the SpinTAC Velocity Controller Object
//! \param[in]  inertia System inertia { unit: [PU/(pu/s^2)], Value range: positive _IQ24 value }
static inline void STVELCTL_setInertia(ST_VELCTL_Handle handle, _iq24 inertia) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->Inertia = inertia;

	return;
} // end of STVELCTL_setInertia function

//! \brief      Gets the Inertia (Inertia) for SpinTAC Velocity Controller
//! \param[in]  handle        The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 Inertia System inertia { unit: [PU/(pu/s^2)], Value range: positive _IQ24 value }
static inline _iq24 STVELCTL_getInertia(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return(obj->Inertia);
} // end of STVELCTL_getInertia function

//! \brief      Sets the Friction (Friction) for SpinTAC Velocity Controller
//! \param[in]  handle   The handle for the SpinTAC Velocity Controller Object
//! \param[in]  friction Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
static inline void STVELCTL_setFriction(ST_VELCTL_Handle handle, _iq24 friction) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->Friction = friction;

	return;
} // end of STVELCTL_setFriction function

//! \brief      Gets the Friction (Friction) from SpinTAC Velocity Controller
//! \param[in]  handle         The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 Friction Friction Coefficient { unit: [PU/(pu/s)], value range: positive _IQ24 value }
static inline _iq24 STVELCTL_getFriction(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return(obj->Friction);
} // end of STVELCTL_getFriction function

//! \brief      Sets the Bandwidth (Bw_radps) for SpinTAC Velocity Controller
//! \param[in]  handle  The handle for the SpinTAC Velocity Controller Object
//! \param[in]  bw      Controller Bandwidth { unit: [rad/s] }
static inline void STVELCTL_setBandwidth_radps(ST_VELCTL_Handle handle, _iq20 bw) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->Bw_radps = bw;

	return;
} // end of STVELCTL_setBandwidth_radps function

//! \brief      Gets the Bandwidth (Bw_radps) for SpinTAC Velocity Controller
//! \param[in]  handle         The handle for the SpinTAC Velocity Controller Object
//! \return     _iq20 Bw_radps Controller Bandwidth {unit: [rad/s] }
static inline _iq20 STVELCTL_getBandwidth_radps(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return(obj->Bw_radps);
} // end of STVELCTL_getBandwidth_radps function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  enb    Enable bit { false: disable; true: enable }
static inline void STVELCTL_setEnable(ST_VELCTL_Handle handle, bool enb) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->ENB = enb;

	return;
} // end of STVELCTL_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Velocity Controller
//! \param[in]  handle   The handle for the SpinTAC Velocity Controller Object
//! \return     bool ENB Enable bit { false: disable; true: enable }
static inline bool STVELCTL_getEnable(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return (obj->ENB);

} // end of STVELCTL_getEnable function

//! \brief      Sets the Torque (Iq) Reference (Out) for SpinTAC Velocity Controller
//! \param[in]  handle The handle for the SpinTAC Velocity Controller Object
//! \param[in]  out    Control output { unit: [PU], value range: [cfg.OutMin, cfg.OutMax] }
static inline void STVELCTL_setTorqueReference(ST_VELCTL_Handle handle, _iq24 out) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	obj->Out = out;

	return;
} // end of STVELCTL_setTorqueReference function

//! \brief      Gets the Torque (Iq) Reference (Out) for SpinTAC Velocity Controller
//! \param[in]  handle    The handle for the SpinTAC Velocity Controller Object
//! \return     _iq24 Out Control output { unit: [PU], value range: [cfg.OutMin, cfg.OutMax] }
static inline _iq24 STVELCTL_getTorqueReference(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return (obj->Out);
} // end of STVELCTL_getTorqueReference function

//! \brief      Gets the Status value (STATUS) for SpinTAC Velocity Controller
//! \param[in]  handle                The handle for the SpinTAC Velocity Controller Object
//! \return     ST_CtlStatus_e STATUS Status { ST_CTL_IDLE, ST_CTL_INIT, ST_CTL_CONF, ST_CTL_BUSY }
static inline ST_CtlStatus_e STVELCTL_getStatus(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return (obj->STATUS);

} // end of STVELCTL_getStatus function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Velocity Controller
//! \param[in]  handle         The handle for the SpinTAC Velocity Controller Object
//! \return     uint16_t error Error ID { 0: no error; others: see error code }
static inline uint16_t STVELCTL_getErrorID(ST_VELCTL_Handle handle) {
	ST_VelCtl_t *obj = (ST_VelCtl_t *)handle;

	return (obj->ERR_ID);

} // end of STVELCTL_getErrorID function

//! \brief      Initializes the SpinTAC Velocity Controller object
//! \param[in]   *pMemory                Pointer to the memory for ST_VelCtl_t
//! \param[in]   numBytes                The number of bytes in the ST_VelCtl_t
//! \return      ST_VELCTL_Handle handle The handle for the SpinTAC Velocity Controller Object
ST_VELCTL_Handle STVELCTL_init(void *pMemory, const size_t numBytes);

//! \brief      Runs the SpinTAC Velocity Control Function
//! \param[in]  handle The Velocity Controller object handle
void STVELCTL_run(ST_VELCTL_Handle handle);	// SpinTAC Velocity Controller function

//@} // defgroup
#endif //__SPINTAC_VEL_CTL_H__
