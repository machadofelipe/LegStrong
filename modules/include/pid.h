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
#ifndef _PID_H_
#define _PID_H_

//! \file   modules/pid/src/32b/pid.h
//! \brief  Contains the public interface to the 
//!         Proportional-Integral-Derivative (PID) controller module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

//modules
#include "IQmathLib.h"
#include "types.h"


//!
//!
//! \defgroup PID PID
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup PID_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the PID controller object
//!
typedef struct _PID_Obj_
{
  _iq Kp;                     //!< the proportional gain for the PID controller
  _iq Ki;                     //!< the integral gain for the PID controller
  _iq Kd;                     //!< the derivative gain for the PID controller

  _iq Ui;                     //!< the integrator start value for the PID controller

  _iq refValue;               //!< the reference input value
  _iq fbackValue;             //!< the feedback input value
		
  _iq outMin;                 //!< the minimum output value allowed for the PID controller
  _iq outMax;                 //!< the maximum output value allowed for the PID controller

} PID_Obj; 


//! \brief Defines the PID handle
//!
typedef struct _PID_Obj_ *PID_Handle;


// **************************************************************************
// the function prototypes

//! \brief     Gets the feedback value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The feedback value in the PID controller
static inline _iq PID_getFbackValue(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->fbackValue);
} // end of PID_getFbackValue() function


//! \brief      Gets the gains in the PID controller
//! \param[in]  handle  The PID controller handle
//! \param[out] pKp     The pointer to the proportional gain value
//! \param[out] pKi     The pointer to the integrator gain value
//! \param[out] pKd     The pointer to the derivative gain value
static inline void PID_getGains(PID_Handle handle,_iq *pKp,_iq *pKi,_iq *pKd)
{
  PID_Obj *obj = (PID_Obj *)handle;

  *pKp = obj->Kp;
  *pKi = obj->Ki;
  *pKd = obj->Kd;

  return;
} // end of PID_getGains() function


//! \brief     Gets the derivative gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The derivative gain in the PID controller
static inline _iq PID_getKd(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Kd);
} // end of PID_getKd() function


//! \brief     Gets the integral gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The integral gain in the PID controller
static inline _iq PID_getKi(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Ki);
} // end of PID_getKi() function


//! \brief     Gets the proportional gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The proportional gain in the PID controller
static inline _iq PID_getKp(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Kp);
} // end of PID_getKp() function


//! \brief      Gets the minimum and maximum output value allowed in the PID controller
//! \param[in]  handle  The PID controller handle
//! \param[out] pOutMin    The pointer to the minimum output value allowed
//! \param[out] pOutMax    The pointer to the maximum output value allowed
static inline void PID_getMinMax(PID_Handle handle,_iq *pOutMin,_iq *pOutMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  *pOutMin = obj->outMin;
  *pOutMax = obj->outMax;

  return;
} // end of PID_getMinMax() function


//! \brief      Gets the maximum output value allowed in the PID controller
//! \param[in]  handle  The PID controller handle
//! \return     The maximum output value allowed
static inline _iq PID_getOutMax(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->outMax);
} // end of PID_getOutMax() function


//! \brief      Gets the minimum output value allowed in the PID controller
//! \param[in]  handle  The PID controller handle
//! \return     The minimum output value allowed
static inline _iq PID_getOutMin(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->outMin);
} // end of PID_getOutMin() function


//! \brief     Gets the reference value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The reference value in the PID controller
static inline _iq PID_getRefValue(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->refValue);
} // end of PID_getRefValue() function


//! \brief     Gets the integrator start value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The integrator start value for the PID controller
static inline _iq PID_getUi(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Ui);
} // end of PID_getUi() function


//! \brief     Initializes the PID controller
//! \param[in] pMemory   A pointer to the memory for the PID controller object
//! \param[in] numBytes  The number of bytes allocated for the PID controller object, bytes
//! \return The PID controller (PID) object handle
extern PID_Handle PID_init(void *pMemory,const size_t numBytes);


//! \brief     Runs the PID controller
//! \param[in] handle      The PID controller handle
//! \param[in] refValue    The reference value to the controller
//! \param[in] fbackValue  The feedback value to the controller
//! \param[in] pOutValue   The pointer to the controller output value
static inline void PID_run(PID_Handle handle,const _iq refValue,const _iq fbackValue,_iq *pOutValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  _iq Error;
  _iq Up,Ui;


  Error = refValue - fbackValue;

  Ui = obj->Ui;                                                  // load the previous integral output
  Up = _IQmpy(obj->Kp,Error);                                    // Compute the proportional output
  Ui = _IQsat(Ui + _IQmpy(obj->Ki,Up),obj->outMax,obj->outMin);  // Compute the integral output

  obj->Ui = Ui;                                                  // store the intetral output
  obj->refValue = refValue;
  obj->fbackValue = fbackValue;

  *pOutValue = _IQsat(Up + Ui,obj->outMax,obj->outMin);          // Saturate the output

  return;
} // end of PID_run() function


//! \brief     Runs the PID controller for speed
//! \param[in] handle      The PID controller handle
//! \param[in] refValue    The reference value to the controller
//! \param[in] fbackValue  The feedback value to the controller
//! \param[in] pOutValue   The pointer to the controller output value
static inline void PID_run_spd(PID_Handle handle,const _iq refValue,const _iq fbackValue,_iq *pOutValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  _iq Error;
  _iq Up,Ui;

  Error = refValue - fbackValue;

  Ui = obj->Ui;                                                     // load the previous integral output
  Up = _IQmpy(obj->Kp,Error);                                       // Compute the proportional output
  Ui = _IQsat(Ui + _IQmpy(obj->Ki,Error),obj->outMax,obj->outMin);  // Compute the integral output

  obj->Ui = Ui;                                                     // store the intetral output
  obj->refValue = refValue;
  obj->fbackValue = fbackValue;

  *pOutValue = _IQsat(Up + Ui,obj->outMax,obj->outMin);             // Saturate the output

  return;
} // end of PID_run_spd() function


//! \brief     Sets the feedback value in the PID controller
//! \param[in] handle      The PID controller handle
//! \param[in] fbackValue  The feedback value
static inline void PID_setFbackValue(PID_Handle handle,const _iq fbackValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->fbackValue = fbackValue;

  return;
} // end of PID_setFbackValue() function


//! \brief     Sets the gains in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Kp      The proportional gain for the PID controller
//! \param[in] Ki      The integrator gain for the PID controller
//! \param[in] Kd      The derivative gain for the PID controller
static inline void PID_setGains(PID_Handle handle,const _iq Kp,const _iq Ki,const _iq Kd)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Kp = Kp;
  obj->Ki = Ki;
  obj->Kd = Kd;

  return;
} // end of PID_setGains() function


//! \brief     Sets the derivative gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Kd      The derivative gain for the PID controller
static inline void PID_setKd(PID_Handle handle,const _iq Kd)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Kd = Kd;

  return;
} // end of PID_setKd() function


//! \brief     Sets the integral gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Ki      The integral gain for the PID controller
static inline void PID_setKi(PID_Handle handle,const _iq Ki)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Ki = Ki;

  return;
} // end of PID_setKi() function


//! \brief     Sets the proportional gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Kp      The proportional gain for the PID controller
static inline void PID_setKp(PID_Handle handle,const _iq Kp)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Kp = Kp;

  return;
} // end of PID_setKp() function


//! \brief     Sets the minimum and maximum output value allowed in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] outMin  The minimum output value allowed
//! \param[in] outMax  The maximum output value allowed
static inline void PID_setMinMax(PID_Handle handle,const _iq outMin,const _iq outMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->outMin = outMin;
  obj->outMax = outMax;

  return;
} // end of PID_setMinMax() function


//! \brief     Sets the maximum output value allowed in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] outMax  The maximum output value allowed
static inline void PID_setOutMax(PID_Handle handle,const _iq outMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->outMax = outMax;

  return;
} // end of PID_setOutMax() function


//! \brief     Sets the minimum output value allowed in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] outMax  The minimum output value allowed
static inline void PID_setOutMin(PID_Handle handle,const _iq outMin)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->outMin = outMin;

  return;
} // end of PID_setOutMin() function


//! \brief     Sets the reference value in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] refValue   The reference value
static inline void PID_setRefValue(PID_Handle handle,const _iq refValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->refValue = refValue;

  return;
} // end of PID_setRefValue() function


//! \brief     Sets the integrator start value in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Ui         The integral start value for the PID controller
static inline void PID_setUi(PID_Handle handle,const _iq Ui)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Ui = Ui;

  return;
} // end of PID_setUi() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif //end of _PID_H_ definition

