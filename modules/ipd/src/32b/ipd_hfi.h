/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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

#ifndef _IPD_HFI_H_
#define _IPD_HFI_H_

//! \file   ~/dmc_rdc/sw/modules/ipd/src/32b/ipd_hfi.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         initial position detection, high frequency injection (IPD) routines
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "types.h"
#include "IQmathLib.h"

#include "filter_fo.h"
#include "math.h"
#include "park.h"
#include "pid.h"


//!
//!
//! \defgroup IPD_HFI IPD_HFI
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup IPD_HFI_OVERVIEW


// **************************************************************************
// the typedefs


//! \brief Defines the initial position detection, high frequency injection (IPD_HFI) states
//!
typedef enum
{
  IPD_HFI_State_Error = 0,     //!< the error state
  IPD_HFI_State_Idle = 1,      //!< the idle state
  IPD_HFI_State_Coarse = 2,    //!< the coarse angle detection state
  IPD_HFI_State_Fine = 3,      //!< the fine angle detection state
  IPD_HFI_State_OnLine = 4,    //!< the online state
  IPD_HFI_State_NumStates = 5  //!< the total number of states
} IPD_HFI_State_e;


//! \brief Defines the trajectory states
//!
typedef enum
{
  IPD_HFI_TRAJ_State_Idle = 0,      //!< the idle trajectory state
  IPD_HFI_TRAJ_State_Coarse = 1,    //!< the coarse trajectory state
  IPD_HFI_TRAJ_State_Fine = 2,      //!< the fine trajectory state
  IPD_HFI_TRAJ_State_NumStates = 3  //!< the total number of states
} IPD_HFI_TRAJ_State_e;


//! \brief Defines the IPD_HFI handle
//!
typedef struct IPD_HFI_Obj  *IPD_HFI_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the functions


//! \brief      Computes a phasor for a given angle
//! \param[in]  angle_pu  The angle, pu
//! \param[out] pPhasor   The pointer to the phasor vector values
static inline void IPD_HFI_computePhasor(const _iq angle_pu,MATH_vec2 *pPhasor)
{

  pPhasor->value[0] = _IQcosPU(angle_pu);
  pPhasor->value[1] = _IQsinPU(angle_pu);

  return;
} // end of IPD_HFI_computePhasor() function


//! \brief     Disables the initial position detection, high frequency injection (IPD_HFI) module
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
extern void IPD_HFI_disable(IPD_HFI_Handle handle);


//! \brief     Enables the initial position detection, high frequency injection (IPD_HFI) module
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
extern void IPD_HFI_enable(IPD_HFI_Handle handle);


//! \brief     Gets the angle value from the estimator
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \return    The angle value, pu
extern _iq IPD_HFI_getAngle_pu(IPD_HFI_Handle handle);


//! \brief     Gets the speed gain value
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \return    The speed gain value, pu
extern _iq IPD_HFI_getKspd_pu(IPD_HFI_Handle handle);


//! \brief     Gets the speed value from the estimator
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \return    The speed value, pu
extern _iq IPD_HFI_getSpeed_pu(IPD_HFI_Handle handle);


//! \brief     Gets the low pass filtered speed value from the estimator
//! \param[in] handle    The initial position detection, high frequency injection (IPD_HFI) handle
//! \return    The low pass filtered speed value, pu
extern _iq IPD_HFI_getSpeed_lp_pu(IPD_HFI_Handle handle);



//! \brief     Gets the estimator state
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \return    The estimator state
extern IPD_HFI_State_e IPD_HFI_getState(IPD_HFI_Handle handle);


//! \brief     Gets the current trajectory magnitude value
//! \param[in] handle  The initial position detection, high frequency injection trajectory generation (IPD_HFI_TRAJ) handle
//! \return    The current trajectory magnitude value, pu
extern _iq IPD_HFI_getVdValue(IPD_HFI_Handle handle);


//! \brief     Initializes the initial position detection, high frequency injection (IPD_HFI) module
//! \param[in] pMemory   A pointer to the memory for the object
//! \param[in] numBytes  The number of bytes allocated for the object, bytes
//! \return The initial position detection, high frequency injection (IPD_HFI) object handle
extern IPD_HFI_Handle IPD_HFI_init(void);


//! \brief     Returns a boolean value denoting if the module is enabled (true) or not (false)
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \return    The boolean value
extern bool IPD_HFI_isEnabled(IPD_HFI_Handle handle);


//! \brief     Denotes whether the module is online (true) or not (false)
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
extern bool IPD_HFI_isOnLine(IPD_HFI_Handle handle);


//! \brief     Runs the initial position detection, high frequency injection (IPD_HFI) algorithm
//! \param[in] handle   The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] pIab_pu  The pointer to the alpha/beta current values, pu
extern void IPD_HFI_run(IPD_HFI_Handle handle,const MATH_vec2 *pIab_pu);


//! \brief     Sets the angle value in the module
//! \param[in] handle    The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] angle_pu  The desired angle value, pu
extern void IPD_HFI_setAngle_pu(IPD_HFI_Handle handle,const _iq angle_pu);


//! \brief     Sets the value of the motor spinning flag
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] value   The desired flag value
void IPD_HFI_setFlag_motorSpinning(IPD_HFI_Handle handle,const bool value);


//! \brief     Sets the Idq high pass filter parameters
//!
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
//!
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] b0      The filter coefficient value for z^0
//! \param[in] b1      The filter coefficient value for z^(-1)
//! \param[in] a1      The filter coefficient value for z^(-1)
//! \param[in] x1      The input value at time sample n=-1
//! \param[in] y1      The output value at time sample n=-1
extern void IPD_HFI_setHpf_Idq_Params(IPD_HFI_Handle handle,
                                      const _iq b0,const _iq b1,const _iq a1,
                                      const _iq x1,const _iq y1);


//! \brief     Sets the integral value for the direct current in the estimator
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] Id_sum  The desired integral value, pu
extern void IPD_HFI_setId_sum(IPD_HFI_Handle handle,const _iq Id_sum_pu);									  


//! \brief     Sets the speed gain value
//! \param[in] handle   The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] Kspd_pu  The desired speed gain value, pu
extern void IPD_HFI_setKspd_pu(IPD_HFI_Handle handle,const _iq Kspd_pu);


//! \brief     Sets the speed low pass filter parameters
//!
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
//!
//! \param[in] handle  The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] b0      The filter coefficient value for z^0
//! \param[in] b1      The filter coefficient value for z^(-1)
//! \param[in] a1      The filter coefficient value for z^(-1)
//! \param[in] x1      The input value at time sample n=-1
//! \param[in] y1      The output value at time sample n=-1
extern void IPD_HFI_setLpf_spd_Params(IPD_HFI_Handle handle,
                                      const _iq b0,const _iq b1,const _iq a1,
                                      const _iq x1,const _iq y1);
									  
									  
//! \brief     Sets the initial position detection, high frequency injection (IPD_HFI) parameters
//! \param[in] handle                 The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] estFreq_Hz             The estimation frequency of the IPD algorithm, Hz
//! \param[in] excFreq_Hz             The excitation frequency of the IPD algorithm, Hz
//! \param[in] lpFilterCutOffFreq_Hz  The lowpass filter cutoff frequency, Hz
//! \param[in] hpFilterCutOffFreq_Hz  The highpass filter cutoff frequency, Hz
//! \param[in] iqFullScaleFreq_Hz     The IQ full scale frequency, Hz
//! \param[in] Kspd                   The speed gain value
//! \param[in] excMag_coarse_pu       The excitation magnitude during coarse position detection, Hz
//! \param[in] excMag_fine_pu         The excitation magnitude during fine position detection, Hz
//! \param[in] waitTime_coarse_sec    The wait time for coarse position detection, sec
//! \param[in] waitTime_fine_sec      The wait time for fine position detection, sec
extern void IPD_HFI_setParams(IPD_HFI_Handle handle,
                              float_t estFreq_Hz,
                              float_t excFreq_Hz,
                              float_t lpFilterCutOffFreq_Hz,
                              float_t hpFilterCutOffFreq_Hz,
                              float_t iqFullScaleFreq_Hz,
                              float_t Kspd,
                              float_t excMag_coarse_pu,
                              float_t excMag_fine_pu,
                              float_t waitTime_coarse_sec,
                              float_t waitTime_fine_sec);

							  
//! \brief     Sets the speed value in the estimator
//! \param[in] handle    The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] speed_lp  The desired speed value, pu
extern void IPD_HFI_setSpeed_pu(IPD_HFI_Handle handle,const _iq speed_pu);							  
							  

//! \brief     Sets the low pass filtered speed value in the estimator
//! \param[in] handle    The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] speed_lp  The desired low pass filtered speed value, pu
extern void IPD_HFI_setSpeed_lp_pu(IPD_HFI_Handle handle,const _iq speed_lp_pu);


//! \brief     Sets the trajectory magnitudes in the estimator
//! \param[in] handle        The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] period        The trajectory period for the current state, counts
//! \param[in] pTrajMags     The pointer to the trajectory magnitude values for each trajectory state, counts
extern void IPD_HFI_setTrajMags(IPD_HFI_Handle handle,const _iq *pMags);


//! \brief     Sets the trajectory parameters in the estimator
//! \param[in] handle        The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] period        The trajectory period for the current state, counts
//! \param[in] pTrajMags     The pointer to the trajectory magnitude values for each trajectory state, counts
//! \param[in] pTrajPeriods  The pointer to the trajectory period values for each trajectory state, counts
extern void IPD_HFI_setTrajParams(IPD_HFI_Handle handle,
                                  const uint_least32_t period,const uint_least32_t targetPeriod,
                                  const _iq *pTrajMags,const uint_least32_t *pTrajPeriods);


//! \brief     Sets the periods for each trajectory state
//! \param[in] handle    The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] pPeriods  The pointer to the trajectory period values for each trajectory state, counts
extern void IPD_HFI_setTrajPeriods(IPD_HFI_Handle handle,const uint_least32_t *pPeriods);								  


//! \brief     Sets the wait times for each estimator state
//! \param[in] handle      The initial position detection, high frequency injection (IPD_HFI) handle
//! \param[in] pWaitTimes  The pointer to the wait times, counts
extern void IPD_HFI_setWaitTimes(IPD_HFI_Handle handle,const uint_least32_t *pWaitTimes);


//! \brief      Updates the state
//! \param[in]  handle  The initial position detection, high frequency injection (IPD_HFI) handle
extern void IPD_HFI_updateState(IPD_HFI_Handle handle);

							  
// end of file

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _IPD_HFI_H_ definition
