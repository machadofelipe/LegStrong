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
#ifndef _OFFSET_H_
#define _OFFSET_H_

//! \file   modules/offset/src/32b/offset.h
//! \brief  Contains the public interface to the 
//!         offset (OFFSET) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the module includes

// modules
#include "types.h"
#include "IQmathLib.h"
#include "filter_fo.h"


//!
//!
//! \defgroup OFFSET OFFSET
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs
 
//! \brief Defines the offset (OFFSET) object
//!
#ifdef FAST_ROM_V1p6
typedef struct _OFFSET_
{

  _iq                value;          //!< the offset value (iq)
  FILTER_FO_Obj      filter;         //!< the first order filter object used.
  FILTER_FO_Handle   filterHandle;   //!< the first order filter handle.

} OFFSET_Obj;
#else
typedef struct _OFFSET_
{

  _iq                value;          //!< the offset value (iq)
  _iq                bias;           //!< the bias value (iq)
  FILTER_FO_Obj      filter;         //!< the first order filter object used.
  FILTER_FO_Handle   filterHandle;   //!< the first order filter handle.

} OFFSET_Obj;
#endif                                                         

//! \brief Defines the OFFSET handle
//!
typedef struct _OFFSET_Obj_ *OFFSET_Handle;
                                                         

// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Gets the beta offset filter coefficient
//! \param[in] handle  The offset handle
//! \return The filter coefficient beta
extern _iq OFFSET_getBeta(OFFSET_Handle handle);


//! \brief     Gets the offset value
//! \param[in] handle  The offset handle
//! \return    The offset value
static inline _iq OFFSET_getOffset(OFFSET_Handle handle)
{
	OFFSET_Obj *obj = (OFFSET_Obj *)handle;

	return(obj->value);
} // end of OFFSET_getOffset() function


//! \brief     Initializes the offset
//! \param[in] pMemory   A pointer to the memory for the offset object
//! \param[in] numBytes  The number of bytes allocated for the offset object, bytes
//! \return The offset (OFFSET) object handle
extern OFFSET_Handle OFFSET_init(void *pMemory,const size_t numBytes);


//! \brief     Runs an offset filter of the form
//!            y[n] = beta*(x[n]+bias) + (1 - beta)*y[n-1]
//!            y -> The DC offset
//!            x -> The ADC measurement
//!
//! \param[in] handle  The offset handle
//! \param[in] inputValue    The input value to offset filter
static inline void OFFSET_run(OFFSET_Handle handle,const _iq inputValue)
{
  OFFSET_Obj *obj = (OFFSET_Obj *)handle;

  obj->value = FILTER_FO_run(obj->filterHandle,inputValue);

  return;
} // end of OFFSET_run() function


//! \brief     Sets the beta offset filter coefficient
//! \param[in] handle  The offset handle
//! \param[in] beta          The offset filter coefficient beta
extern void OFFSET_setBeta(OFFSET_Handle handle,const _iq beta);


//! \brief     Set the initial condition of the integrator or the value of y[n-1]
//! \param[in] handle  The offset handle
//! \param[in] initCond      The mean value that the filter will approximate to
extern void OFFSET_setInitCond(OFFSET_Handle handle,const _iq initCond);


//! \brief     Sets the offset value
//! \param[in] handle  The offset handle
//! \param[in] offsetValue   The offset value
extern void OFFSET_setOffset(OFFSET_Handle handle, _iq offsetValue);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _OFFSET_H_ definition


