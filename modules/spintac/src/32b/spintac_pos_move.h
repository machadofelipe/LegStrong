#ifndef __SPINTAC_POS_MOVE_H__
#define __SPINTAC_POS_MOVE_H__
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

//! \file    modules/spintac/src/32b/spintac_pos_move.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Position Move component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACPOSMOVE SpinTAC Position Move
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
//! \brief Enumeration for the Move Curve Mode states
//!
typedef enum
{
  ST_MOVE_CUR_TRAP=0,	//!< Trapezoidal curve
  ST_MOVE_CUR_SCRV,		//!< S-Curve
  ST_MOVE_CUR_STCRV		//!< ST-Curve
} ST_MoveCurveType_e;
#endif //__ST_MOVE_CURVE_TYPE_ENUM__

//! \brief Enumeration for the Profile Mode states
//!
typedef enum
{
  ST_POS_MOVE_VEL_TYPE=0,	//!< velocity-determined position profile
  ST_POS_MOVE_POS_TYPE		//!< position-determined position profile
} ST_PosMoveProfileType_e;

#ifndef __ST_MOVE_STATUS_ENUM__
#define __ST_MOVE_STATUS_ENUM__
//! \brief Enumeration for the Move Status states
//!
typedef enum
{
  ST_MOVE_IDLE=0,	//!< Move is in idle state, holding position
  ST_MOVE_INIT,		//!< Move is in init state, validating configured parameters
  ST_MOVE_CONF,		//!< Move is in conf state, determining the curves
  ST_MOVE_BUSY,		//!< Move is in busy state, providing the curves
  ST_MOVE_HALT		//!< Move is in busy state, ramping velocity to zero
} ST_MoveStatus_e;
#endif //__ST_MOVE_STATUS_ENUM__

//! \brief      Defines the ST_PosMoveCfg_t data
//! \details    The ST_PosMoveCfg_t object contains all configuration parameters
//!				of the Position Move object.
typedef struct {
  //System config parameters: Can be set only once at system startup
  ST_Axis_e Axis;				        //!< Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
  _iq24 T_sec;					        //!< Sample time { unit: [s], value range: (0, 0.01] }
  _iq24 ROMax_mrev;				        //!< Position Rollover bound { unit: [MRev], value range: [2, 100] }
  _iq24 mrev_TO_pu;				        //!< Conversion ratio from mechanical revolution to pu { value range: [0.002, 1.0]) }
  _iq24 HaltAccLim;                     //!< Acceleration Limit during Halt State { unit: [pu/s^2], value range: [0.001, 120.0] }
  _iq20 HaltJrkLim;                     //!< Jerk Limit during Halt State { unit: [pu/s^3], value range: [0.0005, 2000.0] }
  // Config parameters that may be modified
  ST_PosMoveProfileType_e ProfileType;  //!< Sets the profile type { ST_POS_MOVE_VEL_TYPE: velocity; ST_POS_MOVE_POS_TYPE: position }
  ST_MoveCurveType_e CurveType;	        //!< Curve mode { ST_MOVE_CUR_TRAP: Trap; ST_MOVE_CUR_SCRV: s-Curve; ST_MOVE_CUR_STCRV: st-Curve }
  _iq24 VelStart;				        //!< Velocity Start value { unit: [pu/s], value range: [-1.0, 1.0]) }
  _iq24 PosStart_mrev;			        //!< Position Start value { unit: [MRev], value range: [-ROMax, ROMax) }
} ST_PosMoveCfg_t;	// Structure for SpinTAC Move-Position configuration
				

//! \brief      Defines the ST_PosMoveMsg_t data
//! \details    The ST_PosMoveMsg_t object contains all configuration parameters
//!				of the Position Move message.
typedef struct {
  uint32_t ProTime_tick;    //!< Profile time in sub-million ticks { unit: [tick], value range: (0, 1000000) }
  uint32_t ProTime_mtick;	//!< Profile time in million ticks increments { unit: [million tick], value range: (0, uint32_t max] }
  _iq24 ActualVelLim;		//!< Actual maximum velocity of the profile { unit: [pu/s], value range: (0.0, VelLim] }
  _iq24 ActualAccLim;		//!< Actual maximum acceleration of the profile { unit: [pu/s^2], value range: (0.0, AccLim] }
  _iq24 ActualDecLim;		//!< Actual maximum deceleration of the profile { unit: [pu/s^2], value range: (0.0, DecLim] }
  _iq20 ActualJrkLim;		//!< Actual maximum jerk of the profile { unit: [pu/s^3], value range: (0.0, JrkLim] }
} ST_PosMoveMsg_t;	// Structure for SpinTAC Move-Position information

//! \brief      Defines the ST_PosMove_t data
//! \details    The ST_PosMove_t object contains all parameters needed to
//!				perform Position Move
typedef struct {
  /* Configuration Variables */
  ST_PosMoveCfg_t cfg;
  /* Input Variables */
  int32_t PosStepInt_mrev;	//!< Position Step value integer part { unit: [MRev], value range: (-2^31, 2^31) }
  _iq24 PosStepFrac_mrev;	//!< Position Step value fraction part { unit: [MRev], value range: (-1.0, 1.0) }
  _iq24 VelLim;				//!< Velocity Limit { unit: [pu/s], value range: (0.0, 1.0] )
  	  	  	  	  	  	  	//!< Note: VelLim values < 0.001 pu/s may work, but are not guaranteed
  _iq24 AccLim;				//!< Acceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
  _iq24 DecLim;				//!< Deceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
  _iq20 JrkLim;				//!< Jerk Limit { unit: [pu/s^3], value range: [0.0005, 2000.0] }
  _iq24 VelEnd;				//!< Velocity End value { unit: [pu/s], value range: [-1.0, 1.0] }
  // Control bits
  bool ENB;				    //!< Enable bit { false: disabled; true: enabled }
  bool TST;				    //!< Profile test bit { false: Not Testing; true: Testing Mode }
  /* Output Variables */
  int32_t PosRollOver;		//!< Position rollover counts
  _iq24 PosRef_mrev;		//!< Sawtooth Position profile { unit: [MRev], value range [-ROMax_mrev, ROMax_mrev) }
  _iq24 VelRef;				//!< Velocity profile { unit: [pu/s] }
  _iq24 AccRef;				//!< Acceleration profile { unit: [pu/s^2] }
  _iq20 JrkRef;				//!< Jerk profile { unit:; [pu /s^3] }
  // Information structure
  ST_PosMoveMsg_t msg;
  // Information variables
  ST_MoveStatus_e STATUS;	//!< Profile generator status { ST_MOVE_IDLE, ST_MOVE_INIT, ST_MOVE_CONF, ST_MOVE_BUSY, ST_MOVE_HALT }
  uint16_t ERR_ID;			//!< Error ID { 0: no error; others: see error code }
  /* Internal Variables */
  uint32_t s4[66];
} ST_PosMove_t;	// Structure for SpinTAC Position Move

typedef struct _ST_POSMOVE_Handle_ *ST_POSMOVE_Handle; // SpinTAC Position Move Handle

//! \brief      Sets the Axis (cfg.Axis) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  axis   Axis ID { ST_AXIS0: axis 0, ST_AXIS1: axis 1}
static inline void STPOSMOVE_setAxis(ST_POSMOVE_Handle handle, ST_Axis_e axis) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.Axis = axis;
	}

	return;
} // end of STPOSMOVE_setAxis function

//! \brief      Sets the Profile Type (cfg.ProfileType) for SpinTAC Position Move
//! \param[in]  handle      The handle for the SpinTAC Position Move Object
//! \param[in]  profileType Sets the profile type { ST_POS_MOVE_VEL_TYPE: velocity; ST_POS_MOVE_POS_TYPE: position }
static inline void STPOSMOVE_setProfileType(ST_POSMOVE_Handle handle, ST_PosMoveProfileType_e profileType) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.ProfileType = profileType;
	}

	return;
} // end of STPOSMOVE_setProfileType function

//! \brief      Sets the Curve Type (cfg.CurveType) for SpinTAC Position Move
//! \param[in]  handle    The handle for the SpinTAC Position Move Object
//! \param[in]  curveType Curve type { ST_MOVE_CUR_TRAP: Trap; ST_MOVE_CUR_SCRV: s-Curve; ST_MOVE_CUR_STCRV: st-Curve }
static inline void STPOSMOVE_setCurveType(ST_POSMOVE_Handle handle, ST_MoveCurveType_e curveType) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->cfg.CurveType = curveType;

	return;
} // end of STPOSMOVE_setCurveType function

//! \brief      Sets the Sample Time (cfg.T_sec) for SpinTAC Position Move
//! \param[in]  handle     The handle for the SpinTAC Position Move Object
//! \param[in]  sampleTime Sample time { unit: [s], value range: (0, 0.01] }
static inline void STPOSMOVE_setSampleTime_sec(ST_POSMOVE_Handle handle, _iq24 sampleTime) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.T_sec = sampleTime;
	}

	return;
} // end of STPOSMOVE_setSampleTime_sec function

//! \brief      Sets the Mechanical Revolution Maximum (cfg.ROMax_mrev) for SpinTAC Position Move
//! \param[in]  handle    The handle for the SpinTAC Position Move Object
//! \param[in]  mRevROMax Maximum bound for mechanical revolution { unit: [MRev] }
static inline void STPOSMOVE_setMRevMaximum_mrev(ST_POSMOVE_Handle handle, _iq24 mRevROMax) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.ROMax_mrev = mRevROMax;
	}

	return;
} // end of STPOSMOVE_setMRevMaximum_mrev function

//! \brief      Sets the Unit Conversions for SpinTAC Position Move
//! \param[in]  handle     The handle for the SpinTAC Position Converter Object
//! \param[in]  baseFreq   The value that frequency is scaled with in the system { USER_IQ_FULL_SCALE_FREQ_Hz }
//! \param[in]  polePairs  The number of Pole Pairs in the motor { USER_MOTOR_NUM_POLE_PAIRS }
static inline void STPOSMOVE_setUnitConversion(ST_POSMOVE_Handle handle, float_t baseFreq, uint16_t polePairs) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.mrev_TO_pu = _IQ24(((float_t)polePairs) / baseFreq);
	}

	return;
} // end of STPOSCONV_setUnitConversion function

//! \brief      Sets the Halt Limits (cfg.HaltAccLim & cfg.HaltJrkLim) for SpinTAC Position Move
//! \param[in]  handle     The handle for the SpinTAC Position Move Object
//! \param[in]  haltAccLim Acceleration Limit during Halt State { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[in]  haltJrkLim Jerk Limit during Halt State { unit: [pu/s^3], value range: [0.0005, 2000.0] }
static inline void STPOSMOVE_setHaltLimits(ST_POSMOVE_Handle handle, _iq24 haltAccLim, _iq20 haltJrkLim) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.HaltAccLim = haltAccLim;
		obj->cfg.HaltJrkLim = haltJrkLim;
	}

	return;
} // end of STPOSMOVE_setHaltLimits function

//! \brief      Sets the Velocity Start (cfg.VelStart) for SpinTAC Position Move
//! \param[in]  handle   The handle for the SpinTAC Position Move Object
//! \param[in]  velStart Velocity start value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline void STPOSMOVE_setVelocityStart(ST_POSMOVE_Handle handle, _iq24 velStart) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.VelStart = velStart;
	}

	return;
} // end of STPOSMOVE_setVelocityStart function

//! \brief      Gets the Velocity Start (cfg.VelStart) for SpinTAC Position Move
//! \param[in]  handle         The handle for the SpinTAC Position Move Object
//! \return     _iq24 VelStart Velocity start value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline _iq24 STPOSMOVE_getVelocityStart(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->cfg.VelStart);
} // end of STPOSMOVE_getVelocityStart function

//! \brief      Sets the Position Start (cfg.PosStart_mrev) for SpinTAC Position Move
//! \param[in]  handle   The handle for the SpinTAC Position Move Object
//! \param[in]  posStart Position Start value { unit: [MRev], value range: [-ROMax, ROMax] }
static inline void STPOSMOVE_setPositionStart_mrev(ST_POSMOVE_Handle handle, _iq24 posStart) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	if(obj->STATUS == ST_MOVE_IDLE) {
		obj->cfg.PosStart_mrev = posStart;
	}

	return;
} // end of STPOSMOVE_setPositionStart_mrev function

//! \brief      Gets the Position Start (cfg.PosStart_mrev) for SpinTAC Position Move
//! \param[in]  handle              The handle for the SpinTAC Position Move Object
//! \return     _iq24 PosStart_mrev Position Start value { unit: [MRev], value range: [-ROMax, ROMax] }
static inline _iq24 STPOSMOVE_getPositionStart_mrev(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->cfg.PosStart_mrev);
} // end of STPOSMOVE_getPositionStart_mrev function

//! \brief      Gets the Profile Time (msg.ProTime_tick) for SpinTAC Position Move
//! \param[in]  handle        The handle for the SpinTAC Position Move Object
//! \param[out] ProTime_tick  Amount of time profile will take { unit: [tick], value range: (0, 1000000] }
//! \param[out] ProTime_mtick Amount of time profile will take { unit: [million tick], value range: (0, uint32_t max] }
static inline void STPOSMOVE_getProfileTime_tick(ST_POSMOVE_Handle handle, uint32_t *ProTime_tick, uint32_t *ProTime_mtick) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;
	*ProTime_tick = obj->msg.ProTime_tick;
	*ProTime_mtick = obj->msg.ProTime_mtick;
	return;
} // end of STPOSMOVE_getProfileTime_tick function

//! \brief      Gets the Actual Velocity (msg.ActualVelLim) for SpinTAC Position Move
//! \param[in]  handle    The handle for the SpinTAC Position Move Object
//! \return     _iq24 Vel Maximum velocity of the profile { unit: [pu/s], value range: (0.0, VelLim] }
static inline _iq24 STPOSMOVE_getActualVelocity(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->msg.ActualVelLim);
} // end of STPOSMOVE_getActualVelocity function

//! \brief      Gets the Actual Acceleration (msg.ActualAccLim) for SpinTAC Position Move
//! \param[in]  handle    The handle for the SpinTAC Position Move Object
//! \return     _iq24 Acc Maximum acceleration of the profile { unit: [pu/s^2], value range: (0.0, AccLim] }
static inline _iq24 STPOSMOVE_getActualAcceleration(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->msg.ActualAccLim);
} // end of STPOSMOVE_getActualAcceleration function

//! \brief      Gets the Actual Deceleration (msg.ActualDecLim) for SpinTAC Position Move
//! \param[in]  handle    The handle for the SpinTAC Position Move Object
//! \return     _iq24 Dec Maximum deceleration of the profile { unit: [pu/s^2], value range: (0.0, AccLim] }
static inline _iq24 STPOSMOVE_getActualDeceleration(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->msg.ActualDecLim);
} // end of STPOSMOVE_getActualDeceleration function

//! \brief      Gets the Actual Jerk (msg.ActualJrkLim) for SpinTAC Position Move
//! \param[in]  handle    The handle for the SpinTAC Position Move Object
//! \return     _iq20 Jrk Maximum jerk of the profile { unit: [pu/s^3], value range: (0.0, JrkLim] }
static inline _iq20 STPOSMOVE_getActualJerk(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->msg.ActualJrkLim);
} // end of STPOSMOVE_getActualJerk function

//! \brief      Sets the Position Step (posStepInt_mrev, posStepFrac_mrev) for SpinTAC Position Move
//! \param[in]  handle      The handle for the SpinTAC Position Move Object
//! \param[in]  posStepInt  Position Step integer part value { unit: [MRev], value range: 32-bit integer }
//! \param[in]  posStepFrac Position Step fraction part value { unit: [MRev], value range: (-1.0, 1.0) }
static inline void STPOSMOVE_setPositionStep_mrev(ST_POSMOVE_Handle handle, int32_t posStepInt, _iq24 posStepFrac) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;
	obj->PosStepInt_mrev = posStepInt;
	obj->PosStepFrac_mrev = posStepFrac;

	return;
} // end of STPOSMOVE_getPositionStep_mrev function

//! \brief      Gets the Position Step (posStepInt_mrev, posStepFrac_mrev) for SpinTAC Position Move
//! \param[in]  handle       The handle for the SpinTAC Position Move Object
//! \param[out] *posStepInt  Position Step integer part value { unit: [MRev], value range: 32-bit integer }
//! \param[out] *posStepFrac Position Step fraction part value { unit: [MRev], value range: (-1.0, 1.0) }
static inline void STPOSMOVE_getPositionStep_mrev(ST_POSMOVE_Handle handle, int32_t *posStepInt, _iq24 *posStepFrac) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;
	*posStepInt = obj->PosStepInt_mrev;
	*posStepFrac = obj->PosStepFrac_mrev;

	return;
} // end of STPOSMOVE_getPositionStep_mrev function

//! \brief      Sets the Velocity Limit (VelLim) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  velLim Velocity Limit { unit: [pu/s], value range: (0, 1.0]) )
static inline void STPOSMOVE_setVelocityLimit(ST_POSMOVE_Handle handle, _iq24 velLim) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->VelLim = velLim;

	return;
} // end of STPOSMOVE_setVelocityLimit function

//! \brief      Sets the Acceleration Limit (AccLim) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  accLim Acceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
static inline void STPOSMOVE_setAccelerationLimit(ST_POSMOVE_Handle handle, _iq24 accLim) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->AccLim = accLim;

	return;
} // end of STPOSMOVE_setAccelerationLimit function

//! \brief      Sets the Deceleration Limit (DecLim) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  decLim Deceleration Limit { unit: [pu/s^2], value range: [0.001, 120.0] }
static inline void STPOSMOVE_setDecelerationLimit(ST_POSMOVE_Handle handle, _iq24 decLim) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->DecLim = decLim;

	return;
} // end of STPOSMOVE_setDecelerationLimit function

//! \brief      Sets the Jerk Limit (JrkLim) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  jrkLim Jerk Limit { unit: [pu/s^3], value range: [0.001, 2000.0] }
static inline void STPOSMOVE_setJerkLimit(ST_POSMOVE_Handle handle, _iq20 jrkLim) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->JrkLim = jrkLim;

	return;
} // end of STPOSMOVE_setJerkLimit function

//! \brief      Sets the Velocity End (VelEnd) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  velEnd Velocity end value { unit: [pu/s], value range: [-1.0, 1.0] }
static inline void STPOSMOVE_setVelocityEnd(ST_POSMOVE_Handle handle, _iq24 velEnd) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->VelEnd = velEnd;

	return;
} // end of STPOSMOVE_setVelocityEnd function

//! \brief      Gets the Velocity End (VelEnd) for SpinTAC Position Move
//! \param[in]  handle       The handle for the SpinTAC Position Move Object
//! \return     _iq24 VelEnd Velocity end value { unit: [pu/s], value range: [-1.0, 1.0]  }
static inline _iq24 STPOSMOVE_getVelocityEnd(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->VelEnd);
} // end of STPOSMOVE_getVelocityEnd function

//! \brief      Sets the Position RollOver Counts (PosRollOver) for SpinTAC Position Move
//! \param[in]  handle      The handle for the SpinTAC Position Move Object
//! \param[in]  posRollOver Position rollover counts
static inline void STPOSMOVE_setPositionRollOver(ST_POSMOVE_Handle handle, int32_t posRollOver) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->PosRollOver = posRollOver;

	return;
} // end of STPOSMOVE_setPositionRollOver function

//! \brief      Gets the Position RollOver Counts (PosRollOver) for SpinTAC Position Move
//! \param[in]  handle              The handle for the SpinTAC Position Move Object
//! \return     int32_t PosRollOver Position rollover counts
static inline int32_t STPOSMOVE_getPositionRollOver(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->PosRollOver);
} // end of STPOSMOVE_getPositionReference function

//! \brief      Gets the Position Reference (PosRef_mrev) for SpinTAC Position Move
//! \param[in]  handle            The handle for the SpinTAC Position Move Object
//! \return     _iq24 PosRef_mrev Sawtooth Position profile { unit: [MRev], value range [-ROMax, ROMax] }
static inline _iq24 STPOSMOVE_getPositionReference_mrev(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->PosRef_mrev);
} // end of STPOSMOVE_getPositionReference_mrev function

//! \brief      Gets the Velocity Reference (VelRef) for SpinTAC Position Move
//! \param[in]  handle       The handle for the SpinTAC Position Move Object
//! \return     _iq24 VelRef Velocity reference { unit: [pu/s] }
static inline _iq24 STPOSMOVE_getVelocityReference(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->VelRef);
} // end of STPOSMOVE_getVelocityReference function

//! \brief      Gets the Acceleration Reference (AccRef) for SpinTAC Position Move
//! \param[in]  handle       The handle for the SpinTAC Position Move Object
//! \return     _iq24 AccRef Acceleration reference { unit: [pu/s^2] }
static inline _iq24 STPOSMOVE_getAccelerationReference(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->AccRef);
} // end of STPOSMOVE_getAccelerationReference function

//! \brief      Gets the Jerk Reference (JrkRef) for SpinTAC Position Move
//! \param[in]  handle       The handle for the SpinTAC Position Move Object
//! \return     _iq20 JrkRef Jerk reference { unit: [pu/s^3] }
static inline _iq20 STPOSMOVE_getJerkReference(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return(obj->JrkRef);
} // end of STPOSMOVE_getJerkReference function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  enb    Enable bit { false: disable; true: enable }
static inline void STPOSMOVE_setEnable(ST_POSMOVE_Handle handle, bool enb) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->ENB = enb;

	return;
} // end of STPOSMOVE_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Position Move
//! \param[in]  handle   The handle for the SpinTAC Position Move Object
//! \return     bool ENB Enable bit { false: disable; true: enable }
static inline bool STPOSMOVE_getEnable(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return (obj->ENB);
} // end of STPOSMOVE_getEnable function

//! \brief      Sets the Test signal (TST) for SpinTAC Position Move
//! \param[in]  handle The handle for the SpinTAC Position Move Object
//! \param[in]  tst    Profile test bit { false: not testing; true: determining curves }
static inline void STPOSMOVE_setTest(ST_POSMOVE_Handle handle, bool tst) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	obj->TST = tst;

	return;
} // end of STPOSMOVE_setTest function

//! \brief      Gets the Test signal (TST) for SpinTAC Position Move
//! \param[in]  handle   The handle for the SpinTAC Position Move Object
//! \return     bool TST Profile test bit { false: not testing; true: determining curves }
static inline bool STPOSMOVE_getTest(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return (obj->TST);
} // end of STPOSMOVE_getTest function

//! \brief      Gets the Status value (STATUS) for SpinTAC Position Move
//! \param[in]  handle                 The handle for the SpinTAC Position Move Object
//! \return     ST_MoveStatus_e STATUS Profile generator status { ST_MOVE_IDLE, ST_MOVE_INIT, ST_MOVE_CONF, ST_MOVE_BUSY}
static inline ST_MoveStatus_e STPOSMOVE_getStatus(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return (obj->STATUS);
} // end of STPOSMOVE_getStatus function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Position Move
//! \param[in]  handle          The handle for the SpinTAC Position Move Object
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STPOSMOVE_getErrorID(ST_POSMOVE_Handle handle) {
	ST_PosMove_t *obj = (ST_PosMove_t *)handle;

	return (obj->ERR_ID);
} // end of STPOSMOVE_getErrorID function

//! \brief      Initializes the SpinTAC Position Move object
//! \param[in] *pMemory                 Pointer to the memory for ST_PosMove_t
//! \param[in] numBytes                 The number of bytes in the ST_PosMove_t
//! \return    ST_POSMOVE_Handle handle The handle for the SpinTAC Position Move Object
ST_POSMOVE_Handle STPOSMOVE_init(void *pMemory, const size_t numBytes);

//! \brief      Runs the SpinTAC Position Move ISR Function
//! \param[in]  handle The handle to the Position Move structure
void STPOSMOVE_run(ST_POSMOVE_Handle handle);

//@} // defgroup
#endif //__SPINTAC_POS_MOVE_H__
