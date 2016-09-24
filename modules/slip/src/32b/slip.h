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
#ifndef _SLIP_H_
#define _SLIP_H_

//! \file   modules/slip/src/32b/slip.h
//! \brief  Contains the public interface to the 
//!         slip compensation module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include "types.h"
#include "IQmathLib.h"

//!
//!
//! \defgroup SLIP SLIP
//!
//@{

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//! \brief Defines the slip object
//!
typedef struct _SLIP_Obj_
{
  _iq     sample_time;          //!< sample time of the SLIP module
  int32_t enc_elec_angle;		//!< current electrical angle from encoder
  int32_t incremental_slip;     //!< incremental amount of slip per sample time
  int32_t enc_slip_angle;       //!< amount of total slip in Q24
  int32_t enc_magnetic_angle;   //!< current magnetic angle in Q24
} SLIP_Obj;


//! \brief Defines the slip handle
//!
typedef struct _SLIP_Handle_ *SLIP_Handle;


// **************************************************************************
// the function prototypes


//! \brief Returns the current magnetic angle
//! \param[in] slipHandle				Handle to the SLIP object
//! \return								Magnetic angle in Q24
inline _iq SLIP_getMagneticAngle(SLIP_Handle slipHandle) {
	SLIP_Obj *slip = (SLIP_Obj *) slipHandle;

	return slip->enc_magnetic_angle;
}


//! \brief Initializes the encoder object
//! \param[in] pMemory		Memory pointer to object
//! \param[in] numBytes		Object size
//! \return					Object handle
extern SLIP_Handle SLIP_init(void *pMemory,const size_t numBytes);


//! \brief Based on the electrical angle and the incremental slip, calculates the magnetic angle
//! \param[in] slipHandle				Handle to the SLIP object
//! \return								Nothing
extern void SLIP_run(SLIP_Handle slipHandle);


//! \brief Set the electrical angle
//! \param[in] slipHandle				Handle to the SLIP object
//! \param[in] electricalAngle          Current electrical angle in Q24
//! \return                             Nothing
inline void SLIP_setElectricalAngle(SLIP_Handle slipHandle, _iq electricalAngle) {
	SLIP_Obj *slip = (SLIP_Obj *) slipHandle;

	// set the electrical angle
	slip->enc_elec_angle = electricalAngle;

	return;
}


//! \brief Set the amount of slip velocity and calculates the incremental slip
//! \param[in] slipHandle				Handle to the SLIP object
//! \param[in] slipVelocity             Velocity of the slip in ELectrical Revolutions per second
//! \return                             Nothing
inline void SLIP_setSlipVelocity(SLIP_Handle slipHandle, _iq slipVelocity) {
	SLIP_Obj *slip = (SLIP_Obj *) slipHandle;

	// Calcaulte the amount of incremental slip based on the slip velocity & sample time
	slip->incremental_slip = _IQmpy(slipVelocity, slip->sample_time);

	return;
}


//! \brief Initializes slip object parameters
//! \param[in] slipHandle               Handle to the SLIP object
//! \param[in] sampleTime               Sample time that the SLIP object is being called at
//! \return                             Nothing
extern void SLIP_setup(SLIP_Handle slipHandle, _iq sampleTime);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _SLIP_H_ definition

