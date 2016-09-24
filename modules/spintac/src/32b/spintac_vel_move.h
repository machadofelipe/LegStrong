#ifndef __SPINTAC_VEL_MOVE_H__
#define __SPINTAC_VEL_MOVE_H__
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

//! \file    modules/spintac/src/32b/spintac_vel_move.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Velocity Move component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACVELMOVE SpinTAC Velocity Move
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

#ifndef __ST_MOVE_CURVE_TYPE_ENUM__
#define __ST_MOVE_CURVE_TYPE_ENUM__

//! \brief Enumeration for the Move Curve modes
//!
typedef enum
{
  ST_MOVE_CUR_TRAP=0,	//!< Trapazoidal curve
  ST_MOVE_CUR_SCRV,		//!< S-Curve
  ST_MOVE_CUR_STCRV		//!< ST-Curve
} ST_MoveCurveType_e;
#endif //__ST_MOVE_CURVE_TYPE_ENUM__

#ifndef __ST_MOVE_STATUS_ENUM__
#define __ST_MOVE_STATUS_ENUM__

//! \brief Enumeration for the Move Status states
//!
typedef enum
{
  ST_MOVE_IDLE=0,	//!< Move is in idle state, holding velocity
  ST_MOVE_INIT,		//!< Move is in init state, validating configured parameters
  ST_MOVE_CONF,		//!< Move is in conf state, determining the curves
  ST_MOVE_BUSY,		//!< Move is in busy state, providing the curves
  ST_MOVE_HALT		//!< Not used in ST_MOVE_VEL
} ST_MoveStatus_e;
#endif //__ST_MOVE_STATUS_ENUM__

//! \brief      Defines the ST_VelMoveCfg_t data
//! \details    The ST_VelMoveCfg_t object contains all configuration parameters
//!				of the Velocity Move component.
typedef struct {
  //System config parameters: Can be set only once at system startup
  ST_Axis_e Axis;               //!< Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
  _iq24 T_sec;                  //!< Sample time { unit: [s], value range: (0.0, 0.01] }
  _iq20 HaltJrkLim;             //!< Jerk Limit to use during Halt State { unit: [pu/s^3], value range: [0.0005, 2000.0] }
  // Config parameters that may be modified
  ST_MoveCurveType_e CurveType; //!< Curve Type { ST_MOVE_CUR_TRAP: Trap; ST_MOVE_CUR_SCRV: s-Curve; ST_MOVE_CUR_STCRV: st-Curve }
  _iq24 VelStart;			    //!< Velocity start value { unit: [pu/s], value range: [-1.0, 1.0] }
} ST_VelMoveCfg_t;	// Structure for SpinTAC Move-Velocity configuration
				

//! \brief      Defines the ST_VelMoveMsg_t message data
//! \details    The ST_VelMoveMsg_t object contains all configuration parameters
//!				of the Velocity Move message sub component.
typedef struct {
  uint32_t ProTime_tick;	//!< Amount of time profile will take { unit: [tick], value range: (0, uint32_t max] }
  _iq24 ActualAccLim;		//!< Actual maximum acceleration of the profile { unit: [pu/s^2], value range: (0.0, AccLim] }
  _iq20 ActualJrkLim;		//!< Actual maximum jerk of the profile { unit: [pu/s^3], value range: (0.0, JrkLim] }
} ST_VelMoveMsg_t;			//!< Structure for SpinTAC Move-Velocity information

//! \brief      Defines the ST_VelMove_t data
//! \details    The ST_VelMove_t object contains all configuration parameters
//!				of the Velocity Control component.
typedef struct {
  // Configuration structure
  ST_VelMoveCfg_t cfg;
  // Information structure
  ST_VelMoveMsg_t msg;
  /* Input variables */
  _iq24 VelEnd;				//!< Velocity end value { unit: [pu/s], value range: [-1.0, 1.0] }
  _iq24 AccLim;				//!< Acceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
  _iq20 JrkLim;				//!< Jerk Limit { unit: [pu/s^3], value range: [0.0005, 2000.0] }
  // Control bits
  bool ENB;				    //!< Enable bit { false: disabled; true: enabled }
  bool TST;				    //!< Profile test bit { false: Not Testing; true: Testing Mode }
  /* Output variables */
  _iq24 VelRef;				//!< Velocity reference { unit: [pu/s] }
  _iq24 AccRef;				//!< Acceleration reference { unit: [pu/s^2] }
  _iq20 JrkRef;				//!< Jerk reference { unit:; [pu /s^3] }
  // Information variables
  ST_MoveStatus_e STATUS;	//!< Profile generator status { ST_MOVE_IDLE, ST_MOVE_INIT, ST_MOVE_CONF, ST_MOVE_BUSY}
  uint16_t ERR_ID;			//!< Error ID { 0: no error; others: see error code }
  /* Internal variables */
  uint32_t s1[23];
} ST_VelMove_t;	//!< Structure for SpinTAC Move-Velocity

typedef struct _ST_VELMOVE_Handle_ *ST_VELMOVE_Handle; // SpinTAC Velocity Move Handle

//! \brief      Sets the Axis (cfg.Axis) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  axis   Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
static inline void STVELMOVE_setAxis(ST_VELMOVE_Handle handle, ST_Axis_e axis) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.Axis = axis;
	}

	return;
} // end of STVELMOVE_setAxis function

//! \brief      Sets the Curve Type (cfg.CurveType) for SpinTAC Velocity Move
//! \param[in]  handle    The handle for the SpinTAC Velocity Move Object
//! \param[in]  curveType Curve Type { ST_MOVE_CUR_TRAP: Trap; ST_MOVE_CUR_SCRV: s-Curve; ST_MOVE_CUR_STCRV: st-Curve }
static inline void STVELMOVE_setCurveType(ST_VELMOVE_Handle handle, ST_MoveCurveType_e curveType) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->cfg.CurveType = curveType;

	return;
} // end of STVELMOVE_setCurveType function

//! \brief      Gets the Curve Type (cfg.CurveType) for SpinTAC Velocity Move
//! \param[in]  handle                       The handle for the SpinTAC Velocity Move Object
//! \return     ST_MoveCurveType_e CurveType Curve Type { ST_MOVE_CUR_TRAP: Trap; ST_MOVE_CUR_SCRV: s-Curve; ST_MOVE_CUR_STCRV: st-Curve }
static inline ST_MoveCurveType_e STVELMOVE_getCurveType(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->cfg.CurveType);
} // end of STVELMOVE_getCurveType function

//! \brief      Sets the Sample Time (cfg.T_sec) for SpinTAC Velocity Move
//! \param[in]  handle     The handle for the SpinTAC Velocity Move Object
//! \param[in]  sampleTime Sample time { unit: [s], value range: (0.0, 0.01] }
static inline void STVELMOVE_setSampleTime_sec(ST_VELMOVE_Handle handle, _iq24 sampleTime) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.T_sec = sampleTime;
	}

	return;
} // end of STVELMOVE_setSampleTime_sec function

//! \brief      Sets the Halt Limits (cfg.HaltJrkLim) for SpinTAC Velocity Move
//! \param[in]  handle     The handle for the SpinTAC Velocity Move Object
//! \param[in]  haltJrkLim Jerk Limit during Halt State { unit: [pu/s^3], value range: [0.0005, 2000.0] }
static inline void STVELMOVE_setHaltLimits(ST_VELMOVE_Handle handle, _iq20 haltJrkLim) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.HaltJrkLim = haltJrkLim;
	}

	return;
} // end of STVELMOVE_setHaltLimits function

//! \brief      Sets the Velocity Start (cfg.VelStart) for SpinTAC Velocity Move
//! \param[in]  handle   The handle for the SpinTAC Velocity Move Object
//! \param[in]  velStart Velocity start value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline void STVELMOVE_setVelocityStart(ST_VELMOVE_Handle handle, _iq24 velStart) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.VelStart = velStart;
	}

	return;
} // end of STVELMOVE_setVelocityStart function

//! \brief      Gets the Velocity Start (cfg.VelStart) for SpinTAC Velocity Move
//! \param[in]  handle         The handle for the SpinTAC Velocity Move Object
//! \return     _iq24 VelStart Velocity start value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline _iq24 STVELMOVE_getVelocityStart(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->cfg.VelStart);
} // end of STVELMOVE_getVelocityStart function

//! \brief      Gets the Profile Time (msg.ProTime_tick) for SpinTAC Velocity Move
//! \param[in]  handle                The handle for the SpinTAC Velocity Move Object
//! \return     uint32_t ProTime_tick Amount of time profile will take ( unit: [ticks], value range: (0, uint32_t max] }
static inline uint32_t STVELMOVE_getProfileTime_tick(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->msg.ProTime_tick);
} // end of STVELMOVE_getProfileTime_tick function

//! \brief      Gets the Actual Acceleration (msg.ActualAccLim) for current profile
//! \param[in] 	handle                 The handle for the SpinTAC Velocity Move Object
//! \return     _iq24 msg.ActualAccLim Actual acceleration of the profile { unit: [pu/s^2], value range: (0, AccLim] }
static inline _iq24 STVELMOVE_getActualAcceleration(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->msg.ActualAccLim);
} // end of STVELMOVE_getActualAcceleration function

//! \brief      Gets the Actual Jerk (msg.ActualJrkLim) for current pofile
//! \param[in]  handle                The handle for the SpinTAC Velocity Move Object
//! \return     _iq20 msg.ActualJrkLim Actual jerk of the profile { unit: [pu/s^3], value range: (0, JrkLim] }
static inline _iq20 STVELMOVE_getActualJerk(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->msg.ActualJrkLim);
} // end of STVELMOVE_getActualJerk function

//! \brief      Sets the Velocity End (VelEnd) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  velEnd Velocity end value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline void STVELMOVE_setVelocityEnd(ST_VELMOVE_Handle handle, _iq24 velEnd) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->VelEnd = velEnd;

	return;
} // end of STVELMOVE_setVelocityEnd function

//! \brief      Gets the Velocity End (VelEnd) for SpinTAC Velocity Move
//! \param[in]  handle       The handle for the SpinTAC Velocity Move Object
//! \return     _iq24 VelEnd Velocity end value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline _iq24 STVELMOVE_getVelocityEnd(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->VelEnd);
} // end of STVELMOVE_getVelocityEnd function

//! \brief      Sets the Acceleration Limit (AccLim) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  accLim Acceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
static inline void STVELMOVE_setAccelerationLimit(ST_VELMOVE_Handle handle, _iq24 accLim) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->AccLim = accLim;

	return;
} // end of STVELMOVE_setAccelerationLimit function

//! \brief      Gets the Acceleration Limit (AccLim) for SpinTAC Velocity Move
//! \param[in]  handle       The handle for the SpinTAC Velocity Move Object
//! \return     _iq24 AccLim Acceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
static inline _iq24 STVELMOVE_getAccelerationLimit(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->AccLim);
} // end of STVELMOVE_getAccelerationLimit function

//! \brief      Sets the Jerk Limit (JrkLim) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  jrkLim Jerk Limit { unit: [pu/s^3], value range: [0.0005, 2000.0] }
static inline void STVELMOVE_setJerkLimit(ST_VELMOVE_Handle handle, _iq20 jrkLim) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->JrkLim = jrkLim;

	return;
} // end of STVELMOVE_setJerkLimit function

//! \brief      Gets the Jerk Limit (JrkLim) for SpinTAC Velocity Move
//! \param[in]  handle       The handle for the SpinTAC Velocity Move Object
//! \return     _iq20 JrkLim Jerk Limit { unit: [pu/s^3], value range: [0.0005, 2000.0] }
static inline _iq20 STVELMOVE_getJerkLimit(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->JrkLim);
} // end of STVELMOVE_getJerkLimit function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  enb    Enable bit { false: disable; true: enable }
static inline void STVELMOVE_setEnable(ST_VELMOVE_Handle handle, bool enb) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->ENB = enb;

	return;
} // end of STVELMOVE_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Velocity Move
//! \param[in]  handle   The handle for the SpinTAC Velocity Move Object
//! \return     bool ENB Enable bit { false: disable; true: enable }
static inline bool STVELMOVE_getEnable(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return (obj->ENB);
} // end of STVELMOVE_getEnable function

//! \brief      Sets the Test signal (TST) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  tst    Profile test bit { false: not testing; true: no profile output }
static inline void STVELMOVE_setTest(ST_VELMOVE_Handle handle, bool tst) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->TST = tst;

	return;
} // end of STVELMOVE_setTest function

//! \brief      Gets the Test signal (TST) for SpinTAC Velocity Move
//! \param[in]  handle   The handle for the SpinTAC Velocity Move Object
//! \return     bool TST Profile test bit { false: not testing; true: no profile output }
static inline bool STVELMOVE_getTest(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return (obj->TST);
} // end of STVELMOVE_getTest function

//! \brief      Gets the Velocity Reference (VelRef) for SpinTAC Velocity Move
//! \param[in]  handle       The handle for the SpinTAC Velocity Move Object
//! \return     _iq24 VelRef Velocity reference { unit: [pu/s] }
static inline _iq24 STVELMOVE_getVelocityReference(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->VelRef);
} // end of STVELMOVE_getVelocityReference function

//! \brief      Sets the Velocity Reference (VelRef) for SpinTAC Velocity Move
//! \param[in]  handle The handle for the SpinTAC Velocity Move Object
//! \param[in]  velRef Velocity reference { unit: [pu/s] }
static inline void STVELMOVE_setVelocityReference(ST_VELMOVE_Handle handle, _iq24 velRef) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	obj->VelRef = velRef;

	return;
} // end of STVELMOVE_setVelocityReference function

//! \brief      Gets the Acceleration Reference (AccRef) for SpinTAC Velocity Move
//! \param[in]  handle       The handle for the SpinTAC Velocity Move Object
//! \return     _iq24 AccRef Acceleration reference { unit: [pu/s^2] }
static inline _iq24 STVELMOVE_getAccelerationReference(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->AccRef);
} // end of STVELMOVE_getAccelerationReference function

//! \brief      Gets the Jerk Reference (JrkRef) for SpinTAC Velocity Move
//! \param[in]  handle       The handle for the SpinTAC Velocity Move Object
//! \return     _iq20 JrkRef Jerk reference { unit: [pu /s^3] }
static inline _iq20 STVELMOVE_getJerkReference(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return(obj->JrkRef);
} // end of STVELMOVE_getJerkReference function

//! \brief      Gets the Status value (STATUS) for SpinTAC Velocity Move
//! \param[in]  handle                 The handle for the SpinTAC Velocity Move Object
//! \return     ST_MoveStatus_e STATUS Profile generator status { ST_MOVE_IDLE, ST_MOVE_INIT, ST_MOVE_CONF, ST_MOVE_BUSY}
static inline ST_MoveStatus_e STVELMOVE_getStatus(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return (obj->STATUS);
} // end of STVELMOVE_getStatus function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Velocity Move
//! \param[in]  handle          The handle for the SpinTAC Velocity Move Object
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STVELMOVE_getErrorID(ST_VELMOVE_Handle handle) {
	ST_VelMove_t *obj = (ST_VelMove_t *)handle;

	return (obj->ERR_ID);
} // end of STVELMOVE_getErrorID function

//! \brief      Initializes the SpinTAC Velocity Move object
//! \param[in]   *pMemory                 Pointer to the memory for ST_VelMove_t
//! \param[in]   numBytes                 The number of bytes in the ST_VelMove_t
//! \return      ST_VELMOVE_Handle handle The handle for the SpinTAC Velocity Move Object
ST_VELMOVE_Handle STVELMOVE_init(void *pMemory, const size_t numBytes);

//! \brief      Runs the SpinTAC Velocity Move Function
//! \param[in]  handle The pointer to the Velocity Move structure
void STVELMOVE_run(ST_VELMOVE_Handle handle);

//@} // defgroup
#endif //__SPINTAC_VEL_MOVE_H__
