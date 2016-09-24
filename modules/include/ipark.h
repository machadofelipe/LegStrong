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
#ifndef _IPARK_H_
#define _IPARK_H_

//! \file   modules/ipark/src/32b/ipark.h
//! \brief  Contains the public interface to the 
//!         inverse Park transform (IPARK) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "IQmathLib.h"
#include "math.h"


//!
//!
//! \defgroup IPARK IPARK
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup IPARK_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines the IPARK object
//!
typedef struct _IPARK_Obj_
{

  _iq  sinTh;      //!< the sine of the angle between the d,q and the alpha,beta coordinate systems
  _iq  cosTh;      //!< the cosine of the angle between the d,q and the alpha,beta coordinate systems

} IPARK_Obj;


//! \brief Defines the IPARK handle
//!
typedef struct _IPARK_Obj_ *IPARK_Handle;


// **************************************************************************
// the function prototypes

//! \brief     Gets the cosine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The inverse Park transform handle
//! \return    The cosine of the angle
static inline _iq IPARK_getCosTh(IPARK_Handle handle)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  return(obj->cosTh);
} // end of IPARK_getCosTh() function


//! \brief     Gets the cosine/sine phasor for the inverse Park transform
//! \param[in] handle  The inverse Park transform handle
//! \param[in] pPhasor      The pointer to the cosine/sine phasor
static inline void IPARK_getPhasor(IPARK_Handle handle,MATH_vec2 *pPhasor)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  pPhasor->value[0] = obj->cosTh;
  pPhasor->value[1] = obj->sinTh;

  return;
} // end of IPARK_getPhasor() function


//! \brief     Gets the sine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The inverse Park transform handle
//! return     The sine of the angle
static inline _iq IPARK_getSinTh(IPARK_Handle handle)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  return(obj->sinTh);
} // end of IPARK_getSinTh() function


//! \brief     Initializes the inverse Park transform module
//! \param[in] pMemory      A pointer to the memory for the inverse Park object
//! \param[in] numBytes     The number of bytes allocated for the inverse Park object, bytes
//! \return The inverse Park (IPARK) object handle
extern IPARK_Handle IPARK_init(void *pMemory,const size_t numBytes);


//! \brief     Runs the inverse Park transform module
//! \param[in] handle  The inverse Park transform handle
//! \param[in] pInVec       The pointer to the input vector
//! \param[in] pOutVec      The pointer to the output vector
static inline void IPARK_run(IPARK_Handle handle,const MATH_vec2 *pInVec,MATH_vec2 *pOutVec)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  _iq sinTh = obj->sinTh;
  _iq cosTh = obj->cosTh;

  _iq value_0 = pInVec->value[0];
  _iq value_1 = pInVec->value[1];


  pOutVec->value[0] = _IQmpy(value_0,cosTh) - _IQmpy(value_1,sinTh);
  pOutVec->value[1] = _IQmpy(value_1,cosTh) + _IQmpy(value_0,sinTh);

  return;
} // end of IPARK_run() function


//! \brief     Sets the cosine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The inverse Park transform handle
//! \param[in] cosTh   The cosine of the angle
static inline void IPARK_setCosTh(IPARK_Handle handle,const _iq cosTh)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  obj->cosTh = cosTh;

  return;
} // end of IPARK_setCosTh() function


//! \brief     Sets the cosine/sine phasor for the inverse Park transform
//! \param[in] handle   The inverse Park transform handle
//! \param[in] pPhasor  The pointer to the cosine/sine phasor, pu
static inline void IPARK_setPhasor(IPARK_Handle handle,const MATH_vec2 *pPhasor)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  obj->cosTh = pPhasor->value[0];
  obj->sinTh = pPhasor->value[1];

  return;
} // end of IPARK_setPhasor() function


//! \brief     Sets the sine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The inverse Park transform handle
//! \param[in] sinTh   The sine of the angle
static inline void IPARK_setSinTh(IPARK_Handle handle,const _iq sinTh)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  obj->sinTh = sinTh;

  return;
} // end of IPARK_setSinTh() function


//! \brief     Sets up the inverse Park transform module
//! \param[in] handle  The inverse Park transform handle
//! \param[in] Th_pu   The angle between the d,q and the alpha,beta coordinate systems, pu
static inline void IPARK_setup(IPARK_Handle handle,const _iq Th_pu)
{
  IPARK_Obj *obj = (IPARK_Obj *)handle;

  obj->sinTh = _IQsinPU(Th_pu);
  obj->cosTh = _IQcosPU(Th_pu);

  return;
} // end of IPARK_setup() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _IPARK_H_ definition


