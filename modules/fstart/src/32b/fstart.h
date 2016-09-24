/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
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
#ifndef _FSTART_H_
#define _FSTART_H_

//! \file   modules/fstart/src/32b/fstart.h
//! \brief  Contains the public interface to the 
//!         flying start module routines 
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"

//!
//!
//! \defgroup FSTART FSTART
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup FSTART_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs


//! \brief Defines the FStart handle
//!
typedef struct _FStart_Handle_ *FStart_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Initializes the flying start module
//! \return The flying start (FStart) object handle
extern FStart_Handle FStart_init(void);


//! \brief     Runs the flying start module
//! \param[in] handle     The flying start handle
//! \param[in] fm_Hz      The mechanical frequency, Hz
//! \param[in] flux_VpHz  The flux, V/Hz
//! \return Initial condition for the integrator of the Iq current controller, pu
extern _iq FStart_run(FStart_Handle handle,
		      const _iq fm_Hz,
		      const _iq flux_VpHz);


//! \brief     Sets the parameters for the flying start module
//! \param[in] handle                The flying start handle
//! \param[in] iqFullScaleVoltage_V  The IQ full scale voltage, V
//! \param[in] iqFullScaleFreq_Hz    The IQ full scale frequency, Hz
//! \param[in] estFreq_Hz            The estimator frequency, Hz
//! \param[in] maxFlux_VpHz          The maximum per unit flux value for dynamic scaling, V/Hz
extern void FStart_setParams(FStart_Handle handle,
			     const float_t iqFullScaleVoltage_V,
			     const float_t iqFullScaleFreq_Hz,
			     const float_t estFreq_Hz,
			     const float_t maxFlux_VpHz);

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _FSTART_H_ definition

