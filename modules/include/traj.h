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
#ifndef _TRAJ_H_
#define _TRAJ_H_

//! \file   modules/traj/src/32b/traj.h
//! \brief  Contains public interface to various functions related
//!         to the trajectory (TRAJ) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"
#include "IQmathLib.h"

//!
//!
//! \defgroup TRAJ TRAJ
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup TRAJ_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the trajectory (TRAJ) object
//!
typedef struct _TRAJ_Obj_
{
  _iq   targetValue;   //!< the target value for the trajectory
  _iq   intValue;      //!< the intermediate value along the trajectory
  _iq   minValue;      //!< the minimum value for the trajectory generator
  _iq   maxValue;      //!< the maximum value for the trajectory generator
  _iq   maxDelta;      //!< the maximum delta value for the trajectory generator
} TRAJ_Obj;


//! \brief Defines the TRAJ handle
//!
typedef struct _TRAJ_Obj_ *TRAJ_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Gets the intermediate value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The intermediate value
static inline _iq TRAJ_getIntValue(TRAJ_Handle handle)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  return(obj->intValue);
} // end of TRAJ_getIntValue() function


//! \brief     Gets the maximum delta value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The maximum delta value
static inline _iq TRAJ_getMaxDelta(TRAJ_Handle handle)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  return(obj->maxDelta);
} // end of TRAJ_getMaxDelta() function


//! \brief     Gets the maximum value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The maximum value
static inline _iq TRAJ_getMaxValue(TRAJ_Handle handle)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  return(obj->maxValue);
} // end of TRAJ_getMaxValue() function


//! \brief     Gets the minimum value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The minimum value
static inline _iq TRAJ_getMinValue(TRAJ_Handle handle)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  return(obj->minValue);
} // end of TRAJ_getMinValue() function


//! \brief     Gets the target value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The target value
static inline _iq TRAJ_getTargetValue(TRAJ_Handle handle)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  return(obj->targetValue);
} // end of TRAJ_getTargetValue() function


//! \brief     Initializes the trajectory (TRAJ) object
//! \param[in] pMemory   A pointer to the memory for the trajectory (TRAJ) object
//! \param[in] numBytes  The number of bytes allocated for the trajectory object, bytes
//! \return The trajectory (TRAJ) object handle
extern TRAJ_Handle TRAJ_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the intermediate value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] intValue  The intermediate value
static inline void TRAJ_setIntValue(TRAJ_Handle handle,const _iq intValue)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  obj->intValue = intValue;

  return;
} // end of TRAJ_setIntValue() function


//! \brief     Sets the maximum delta value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] maxDelta  The maximum delta value
static inline void TRAJ_setMaxDelta(TRAJ_Handle handle,const _iq maxDelta)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  obj->maxDelta = maxDelta;

  return;
} // end of TRAJ_setMaxDelta() function


//! \brief     Sets the maximum value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] maxValue  The maximum value
static inline void TRAJ_setMaxValue(TRAJ_Handle handle,const _iq maxValue)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  obj->maxValue = maxValue;

  return;
} // end of TRAJ_setMaxValue() function


//! \brief     Sets the minimum value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] minValue  The minimum value
static inline void TRAJ_setMinValue(TRAJ_Handle handle,const _iq minValue)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  obj->minValue = minValue;

  return;
} // end of TRAJ_setMinValue() function


//! \brief     Sets the target value for the trajectory
//! \param[in] handle       The trajectory (TRAJ) handle
//! \param[in] targetValue  The target value
static inline void TRAJ_setTargetValue(TRAJ_Handle handle,const _iq targetValue)
{
  TRAJ_Obj *obj = (TRAJ_Obj *)handle;

  obj->targetValue = targetValue;

  return;
} // end of TRAJ_setTargetValue() function


//! \brief     Runs the trajectory (TRAJ) object
//! \param[in] handle  The trajectory (TRAJ) handle
static inline void TRAJ_run(TRAJ_Handle handle)
{
  _iq targetValue = TRAJ_getTargetValue(handle);
  _iq intValue = TRAJ_getIntValue(handle);
  _iq error = targetValue - intValue;
  _iq maxDelta = TRAJ_getMaxDelta(handle);
  _iq minValue = TRAJ_getMinValue(handle);
  _iq maxValue = TRAJ_getMaxValue(handle);

    // increment the value
    intValue += _IQsat(error,maxDelta,-maxDelta);

    // bound the value
    intValue = _IQsat(intValue,maxValue,minValue);

  // store the value
  TRAJ_setIntValue(handle,intValue);

  return;
} // end of TRAJ_run() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _TRAJ_H_ definition





