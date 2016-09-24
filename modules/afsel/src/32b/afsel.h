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
#ifndef _AFSEL_H_
#define _AFSEL_H_

//! \file   modules/afsel/src/32b/afsel_trq.h
//! \brief  Contains the public interface to the 
//!         Park transform module routines 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "IQmathLib.h"
#include "math.h"
#include "types.h"
#include "est.h"
#include "ipd_hfi.h"


//!
//!
//! \defgroup AFSEL AFSEL
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup AFSEL_OVERVIEW

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs


//! \brief Defines the AFSEL Torque handle
//!
typedef struct _AFSEL_Obj_ *AFSEL_Handle;


// **************************************************************************
// the function prototypes


//! \brief     Disable the afsel module
//! \param[in] pMemory      A pointer to the memory for the afsel object
extern void AFSEL_disable(AFSEL_Handle handle);


//! \brief     Enable the afsel module
//! \param[in] pMemory      A pointer to the memory for the afsel object
extern void AFSEL_enable(AFSEL_Handle handle);


//! \brief     Gets the angle that will currently be used for FOC
//! \param[in] handle  The angle and frequency selector (AFSEL) handle
//! \return            The electrical angle
extern _iq AFSEL_getAngle_pu(AFSEL_Handle handle);


//! \brief     Gets the speed that will currently be used
//! \param[in] handle  The angle and frequency selector (AFSEL) handle
//! \return            The electrical frequency
extern _iq AFSEL_getFreq_pu(AFSEL_Handle handle);


//! \brief     Gets the Iq maximum value currently used
//! \param[in] handle  The angle and frequency selector (AFSEL) handle
//! \return            The maximum Iq current allowed
extern _iq AFSEL_getIqMax(AFSEL_Handle handle);


//! \brief     Gets the current Iq trajectory slope
//! \param[in] handle     The angle and frequency selector (AFSEL) handle
//! \return               The current Iq slope
extern _iq AFSEL_getIqSlope(AFSEL_Handle handle);


//! \brief     Initializes the afsel module
//! \param[in] pMemory      A pointer to the memory for the afsel torque object
//! \param[in] numBytes     The number of bytes allocated for the afsel torque object, bytes
//! \return                 The afsel torque (AFSEL) object handle
extern AFSEL_Handle AFSEL_init(void);


//! \brief     Is the afsel module disabled?
//! \param[in] pMemory      A pointer to the memory for the afsel torque object
//! \return The afsel disabled flag
extern bool AFSEL_isDisabled(AFSEL_Handle handle);


//! \brief     Is the afsel module enabled?
//! \param[in] pMemory      A pointer to the memory for the afsel torque object
//! \return The afsel enabled flag
extern bool AFSEL_isEnabled(AFSEL_Handle handle);


//! \brief     Run the afsel
//! \param[in] handle  The angle and frequency selector (AFSEL) handle
extern void AFSEL_run(AFSEL_Handle handle);


//! \brief     Sets the threshold counter
//! \param[in] handle                         The angle and frequency selector (AFSEL) handle
//! \param[in] value                          The threshold value
static inline void AFSEL_setCountHysThreshold(AFSEL_Handle handle,const uint16_t value);


//! \brief     Sets the frequency at which the HF estimator takes over angle and freq
//! \param[in] handle   The angle and frequency selector (AFSEL) handle
//! \param[in] freq     The frequency when the HF estimator takes control of angle and freq
extern void AFSEL_setFreqHigh_pu(AFSEL_Handle handle,const _iq freq);


//! \brief     Sets the frequency at which the LF estimator takes over angle and freq
//! \param[in] handle   The angle and frequency selector (AFSEL) handle
//! \param[in] freq     The frequency when the LF estimator takes control of angle and freq
extern void AFSEL_setFreqLow_pu(AFSEL_Handle handle,const _iq freq);


//! \brief     Sets the parameters
//! \param[in] handle         The angle and frequency selector (AFSEL) handle
//! \param[in] IqMaxLfEst     The maximum Iq current when the LF estimator is active
//! \param[in] IqMaxHfEst     The maximum Iq current when the HF estimator is active
//! \param[in] IqSlopeLfEst   The trajectory slope of the Iq current reference when the LF estimator is active
//! \param[in] IqSlopeHfEst   The trajectory slope of the Iq current reference when the HF estimator is active
//! \param[in] freqLow_pu     The frequency below which the LF estimator is active
//! \param[in] freqHigh_pu    The frequency above which the HF estimator is active
//! \param[in] hfiHandle      The handle to the HF estimator
//! \param[in] estHandle      The handle to the LF estimator
extern void AFSEL_setParams(AFSEL_Handle handle,
                            const _iq IqMaxLfEst,
                            const _iq IqMaxHfEst,
                            const _iq IqSlopeLfEst,
                            const _iq IqSlopeHfEst,
                            const _iq freqLow_pu,
                            const _iq freqHigh_pu,
                            IPD_HFI_Handle hfiHandle,
                            EST_Handle estHandle);


//! \brief     Sets the parameters in the ISR before AFSEL_run() is called
//! \param[in] handle       The angle and frequency selector (AFSEL) handle
//! \param[in] angleLf_pu   The angle from the LF estimator
//! \param[in] freqLf_pu    The frequency from the LF estimator
//! \param[in] angleHf_pu   The angle from the HF estimator
//! \param[in] freqHf_pu    The frequency from the HF estimator
extern void AFSEL_setup(AFSEL_Handle handle,
                  const _iq angleLf_pu,
                  const _iq freqLf_pu,
                  const _iq angleHf_pu,
                  const _iq freqHf_pu);


//! \brief     Updates the state, called outside of the ISR
//! \param[in] handle                         The angle and frequency selector (AFSEL) handle
extern void AFSEL_updateState(AFSEL_Handle handle);



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _AFSEL_H_ definition

