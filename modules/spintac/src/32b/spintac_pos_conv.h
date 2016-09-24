#ifndef __SPINTAC_POS_CONV_H__
#define __SPINTAC_POS_CONV_H__
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

//! \file    modules/spintac/src/32b/spintac_pos_conv.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Position Convert component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACPOSCONV SpinTAC Position Convert
//@{

#include "spintac_version.h"

//! \brief Enumeration for the Position Convert Status states
//!
typedef enum
{
  ST_POS_CONV_IDLE=0,	//!< Position Converter is in idle state, zero output
  ST_POS_CONV_INIT,		//!< Position Converter is in init state, validating configured parameters
  ST_POS_CONV_BUSY		//!< Position Converter is in busy state, converting position and velocity signals
} ST_PosConvStatus_e;

//! \brief      Defines the ST_PosConvCfg_t data
//! \details    The ST_PosConvCfg_t object contains all configuration parameters
//!				of the Position Convert object.
typedef struct {
  _iq24	T_sec;			          //!< Sample Time { unit: [s], value range: (0, 0.01] }
  _iq24	ROMax_erev;		          //!< Maximum bound for electrical revolution { unit: [ERev] }
  _iq24	ROMin_erev;		          //!< Minimum bound for electrical revolution { unit: [ERev] }
  _iq24	erev_TO_pu_ps;	          //!< Conversion ratio from electrical revolution to user unit
  _iq24	PolePairs;		          //!< Conversion ratio from mechanical revolution to electrical revolution { Pole Pairs }
  _iq24	ROMax_mrev;		          //!< Position Rollover bound { unit: [MRev] }
  int16_t LpfTime_tick;	          //!< Low pass filter ISR ticks 	{ unit: [ticks], value range: [1, 100] }
  // These values are only required for ACIM motors
  _iq24 SampleTimeOverTimeConst;  //!< Scalar value used in the Slip Compensator (only needed for ACIM) { value range: positive _IQ24 value }
  _iq24 OneOverFreqTimeConst;     //!< Scalar value used in the Slip Compensator (only needed for ACIM) { value range: positive _IQ24 value }
} ST_PosConvCfg_t;   	          //!< Structure for SpinTAC Position Converter configuration

//! \brief      Defines the ST_PosConv_t data
//! \details    The ST_PosConv_t object contains all parameters needed to
//!				perform Position Conversion
typedef struct {
  /* Configuration variables */
  ST_PosConvCfg_t cfg;
  /* Input variables */
  _iq24	Pos_erev;			  //!< Electrical angle { unit: [ERev] }
  _iq24 Id;                   //!< the Id current (only needed for ACIM) { unit: [PU] }
  _iq24 Iq;                   //!< the Iq current (only needed for ACIM) { unit: [PU] }
  // Control bits
  bool	ENB;			      //!< Enable bit { false: disable; true: enable }
  /* Output variables */
  _iq24	Vel;				  //!< Speed unfiltered { unit: [pu/s] }
  _iq24	VelLpf;				  //!< Speed filtered { unit: [pu/s] }
  _iq24	Pos_mrev;			  //!< Position { unit: [MRev] }
  int32_t PosROCounts;		  //!< Position rollover counts
  _iq24 SlipVel;              //!< Speed of the magnetic slip (only for ACIM) { unit: [ERev/s] }
  // Information variables
  ST_PosConvStatus_e STATUS;  //!< Status { ST_POS_CONV_IDLE,	ST_POS_CONV_INIT,ST_POS_CONV_BUSY }
  uint16_t	ERR_ID;			  //!< Error ID { 0: no error; others: see error code }
  /* Internal Variables */
  uint32_t s0[19];
} ST_PosConv_t;   	// Structure for SpinTAC Position Converter

typedef struct _ST_POSCONV_Handle_ *ST_POSCONV_Handle; // SpinTAC Position Converter Handle

//! \brief      Sets the Sample Time (cfg.T) for SpinTAC Position Converter
//! \param[in]  handle     The handle for the SpinTAC Position Converter Object
//! \param[in]  sampleTime Sample Time { unit: [s], value range: (0, 0.01] }
static inline void STPOSCONV_setSampleTime_sec(ST_POSCONV_Handle handle, _iq24 sampleTime) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	if(obj->STATUS == ST_POS_CONV_IDLE) {
		obj->cfg.T_sec = sampleTime;
	}

	return;
} // end of STPOSCONV_setSampleTime_sec function

//! \brief      Sets the Electrical Revolution Maximum (cfg.ERevROMax) & Minimum (cfg.ERevROMin) for SpinTAC Position Converter
//! \param[in]  handle    The handle for the SpinTAC Position Converter Object
//! \param[in]  eRevROMax Maximum bound for electrical revolution { unit: [ERev] }
//! \param[in]  eRevROMin Minimum bound for electrical revolution { unit: [ERev] }
static inline void STPOSCONV_setERevMaximums_erev(ST_POSCONV_Handle handle, _iq24 eRevROMax, _iq24 eRevROMin) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	if(obj->STATUS == ST_POS_CONV_IDLE) {
		obj->cfg.ROMax_erev = eRevROMax;
		obj->cfg.ROMin_erev = eRevROMin;
	}

	return;
} // end of STPOSCONV_setERevMaximums_erev function

//! \brief      Sets the Unit Conversions for SpinTAC Position Converter
//! \param[in]  handle     The handle for the SpinTAC Position Converter Object
//! \param[in]  baseFreq   The value that frequency is scaled with in the system { USER_IQ_FULL_SCALE_FREQ_Hz }
//! \param[in]  sampleTime Sample Time { unit: [s], value range: (0, 0.01] }
//! \param[in]  polePairs  The number of Pole Pairs in the motor { USER_MOTOR_NUM_POLE_PAIRS }
static inline void STPOSCONV_setUnitConversion(ST_POSCONV_Handle handle, float_t baseFreq, float_t sampleTime, uint16_t polePairs) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	if(obj->STATUS == ST_POS_CONV_IDLE) {
		obj->cfg.erev_TO_pu_ps = _IQ24(1.0 / (baseFreq * sampleTime));
		obj->cfg.PolePairs = _IQ24(polePairs);
	}

	return;
} // end of STPOSCONV_setUnitConversion function

//! \brief      Sets the Mechanical Revolution Maximum (cfg.ROMax_mrev) for SpinTAC Position Converter
//! \param[in]  handle    The handle for the SpinTAC Position Converter Object
//! \param[in]  mRevROMax Maximum bound for mechanical revolution { unit: [MRev] }
static inline void STPOSCONV_setMRevMaximum_mrev(ST_POSCONV_Handle handle, _iq24 mRevROMax) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	if(obj->STATUS == ST_POS_CONV_IDLE) {
		obj->cfg.ROMax_mrev = mRevROMax;
	}

	return;
} // end of STPOSCONV_setMRevMaximum_mrev function

//! \brief      Sets the Lowpass Time (cfg.LpfTime_tick) for SpinTAC Position Converter
//! \param[in]  handle  The handle for the SpinTAC Position Converter Object
//! \param[in]  lpfTime Low pass filter ISR ticks { unit: [ticks], value range: [1, 100] }
static inline void STPOSCONV_setLowPassFilterTime_tick(ST_POSCONV_Handle handle, int16_t lpfTime) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	if(obj->STATUS == ST_POS_CONV_IDLE) {
		obj->cfg.LpfTime_tick = lpfTime;
	}

	return;
} // end of STPOSCONV_setLowPassFilterTime_tick function

//! \brief      Sets up the Slip Compensator (only for ACIM) for SpinTAC Position Converter
//! \param[in]  handle           The handle for the SpinTAC Position Converter Object
//! \param[in]  sampleTime       Sample Time (float) { unit: [s], value range: (0, 0.01] }
//! \param[in]  baseFreq         The value that frequency is scaled with in the system { USER_IQ_FULL_SCALE_FREQ_Hz }
//! \param[in]  rotorResistance  The resistance of the motor rotor, estimated by FAST { USER_MOTOR_Rr }
//! \param[in]  statorInductance The inductance of the motor stator, estimated by FAST { USER_MOTOR_Ls_d }
static inline void STPOSCONV_setupSlipCompensator(ST_POSCONV_Handle handle, float_t sampleTime, float_t baseFreq, float_t rotorResistance, float_t statorInductance) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	if(obj->STATUS == ST_POS_CONV_IDLE) {
		obj->cfg.SampleTimeOverTimeConst = _IQ24(sampleTime * (rotorResistance/statorInductance));
		obj->cfg.OneOverFreqTimeConst = _IQ24((1.0 / baseFreq) * (rotorResistance/statorInductance));
	}

	return;
} // end of STPOSCONV_setupSlipCompensator function

//! \brief      Sets the Electrical Angle (Pos_erev) for SpinTAC Position Converter
//! \param[in]  handle  The handle for the SpinTAC Position Converter Object
//! \param[in]  posERev Electrical angle { unit: [ERev] }
static inline void STPOSCONV_setElecAngle_erev(ST_POSCONV_Handle handle, _iq24 posERev) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	obj->Pos_erev = posERev;

	return;
} // end of STPOSCONV_setElecAngle_erev function

//! \brief      Sets the Current Vector (IdqIn) (only needed for ACIM) for SpinTAC Position Converter
//! \param[in]  handle  The handle for the SpinTAC Position Converter Object
//! \param[in]  pIdqIn  The vector of the direct/quadrature current input vector values { unit: [PU] }
static inline void STPOSCONV_setCurrentVector(ST_POSCONV_Handle handle, MATH_vec2 *pIdqIn) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	obj->Id = pIdqIn->value[0];
	obj->Iq = pIdqIn->value[1];

	return;
} // end of STPOSCONV_setCurrentVector function

//! \brief      Gets the Velocity (Vel) for SpinTAC Position Converter
//! \param[in]  handle    The handle for the SpinTAC Position Converter Object
//! \return     _iq24 Vel Speed unfiltered { unit: [pu/s] }
static inline _iq24 STPOSCONV_getVelocity(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return(obj->Vel);
} // end of STPOSCONV_getVelocity function

//! \brief      Gets the Velocity Filtered (VelLpf) for SpinTAC Position Converter
//! \param[in]  handle       The handle for the SpinTAC Position Converter Object
//! \return     _iq24 VelLpf Speed filtered { unit: [pu/s] }
static inline _iq24 STPOSCONV_getVelocityFiltered(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return(obj->VelLpf);
} // end of STPOSCONV_getVelocityFiltered function

//! \brief      Gets the Position (Pos_mrev) for SpinTAC Position Converter
//! \param[in]  handle         The handle for the SpinTAC Position Converter Object
//! \return     _iq24 Pos_mrev Position { unit: [MRev] }
static inline _iq24 STPOSCONV_getPosition_mrev(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return(obj->Pos_mrev);
} // end of STPOSCONV_getPosition_mrev function

//! \brief      Gets the Position RollOver Counts (PosROCounts) for SpinTAC Position Converter
//! \param[in]  handle              The handle for the SpinTAC Position Converter Object
//! \return     int32_t PosROCounts Position rollover counts
static inline int32_t STPOSCONV_getPositionRollOver(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return(obj->PosROCounts);
} // end of STPOSCONV_getPositionRollOver function

//! \brief      Gets the Slip Velocity (SlipVel) (only for ACIM) for SpinTAC Position Converter
//! \param[in]  handle        The handle for the SpinTAC Position Converter Object
//! \return     _iq24 SlipVel Speed of the magnetic slip (only for ACIM) { unit: [ERev/s] }
static inline _iq24 STPOSCONV_getSlipVelocity(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return(obj->SlipVel);
} // end of STPOSCONV_getSlipVelocity function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Position Converter
//! \param[in]  handle The handle for the SpinTAC Position Converter Object
//! \param[in]  enb    Enable bit { false: disable; true: enable }
static inline void STPOSCONV_setEnable(ST_POSCONV_Handle handle, bool enb) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	obj->ENB = enb;

	return;
} // end of STPOSCONV_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Position Converter
//! \param[in]  handle   The handle for the SpinTAC Position Converter Object
//! \return     bool ENB Enable bit { false: disable; true: enable }
static inline bool STPOSCONV_getEnable(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return (obj->ENB);
} // end of STPOSCONV_getEnable function

//! \brief      Gets the Status value (STATUS) for SpinTAC Position Converter
//! \param[in]  handle                    The handle for the SpinTAC Position Converter Object
//! \return     ST_PosConvStatus_e STATUS Status { ST_POS_CONV_IDLE, ST_POS_CONV_INIT, ST_POS_CONV_BUSY }
static inline ST_PosConvStatus_e STPOSCONV_getStatus(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return (obj->STATUS);
} // end of STPOSCONV_getStatus function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Position Converter
//! \param[in]  handle          The handle for the SpinTAC Position Converter Object
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STPOSCONV_getErrorID(ST_POSCONV_Handle handle) {
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return (obj->ERR_ID);
} // end of STPOSCONV_getErrorID function

//! \brief      Initializes the SpinTAC Position Converter object
//! \param[in]   *pMemory                 Pointer to the memory for ST_PosConv_t
//! \param[in]   numBytes                 The number of bytes in the ST_PosConv_t
//! \return      ST_POSCONV_Handle handle The handle for the SpinTAC Position Converter Object
ST_POSCONV_Handle STPOSCONV_init(void *pMemory, const size_t numBytes);

//! \brief      Performs Position Roll Over Addition
//! \param[in]  handle The handle for the Position Converter structure
void STPOSCONV_run(ST_POSCONV_Handle handle); 	// SpinTAC Position Converter function
//@} // defgroup
#endif //__SPINTAC_POS_CONV_H__
