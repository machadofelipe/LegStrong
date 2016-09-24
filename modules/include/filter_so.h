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
#ifndef _FILTER_SO_H_
#define _FILTER_SO_H_

//! \file   modules/filter/src/32b/filter_so.h
//! \brief  Contains the public interface to the 
//!         second-order filter (FILTER_SO) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"

#include "IQmathLib.h"


//!
//!
//! \defgroup FILTER_SO FILTER_SO
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs
 
//! \brief Defines the second-order filter (FILTER_SO) object
//!
typedef struct _FILTER_SO_
{    
  _iq     a1;          //!< the denominator filter coefficient value for z^(-1)
  _iq     a2;          //!< the denominator filter coefficient value for z^(-2)

  _iq     b0;          //!< the numerator filter coefficient value for z^0
  _iq     b1;          //!< the numerator filter coefficient value for z^(-1)
  _iq     b2;          //!< the numerator filter coefficient value for z^(-2)

  _iq     x1;          //!< the input value at time sample n=-1
  _iq     x2;          //!< the input value at time sample n=-2

  _iq     y1;          //!< the output value at time sample n=-1
  _iq     y2;          //!< the output value at time sample n=-2
} FILTER_SO_Obj;
                                                         

//! \brief Defines the second-order filter (FILTER_SO) handle
//!
typedef struct _FILTER_SO_Obj_ *FILTER_SO_Handle;                            
                                                         

// **************************************************************************
// the globals


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Gets the second-order filter denominator coefficient a1
//! \param[in] handle  The filter handle
//! \return    The filter coefficient value for z^(-1)
static inline _iq FILTER_SO_get_a1(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->a1);
} // end of FILTER_SO_get_a1() function


//! \brief     Gets the second-order filter denominator coefficient a2
//! \param[in] handle  The filter handle
//! \return    The filter coefficient value for z^(-2)
static inline _iq FILTER_SO_get_a2(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->a2);
} // end of FILTER_SO_get_a2() function


//! \brief     Gets the second-order filter numerator coefficient b0
//! \param[in] handle  The filter handle
//! \return    The filter coefficient value for z^0
static inline _iq FILTER_SO_get_b0(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->b0);
} // end of FILTER_SO_get_b0() function


//! \brief     Gets the second-order filter numerator coefficient b1
//! \param[in] handle  The filter handle
//! \return    The filter coefficient value for z^(-1)
static inline _iq FILTER_SO_get_b1(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->b1);
} // end of FILTER_SO_get_b1() function


//! \brief     Gets the second-order filter numerator coefficient b2
//! \param[in] handle  The filter handle
//! \return    The filter coefficient value for z^(-2)
static inline _iq FILTER_SO_get_b2(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->b2);
} // end of FILTER_SO_get_b2() function


//! \brief     Gets the second-order filter input value at time sample n=-1
//! \param[in] handle  The filter handle
//! \return    The input value at time sample n=-1
static inline _iq FILTER_SO_get_x1(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->x1);
} // end of FILTER_SO_get_x1() function


//! \brief     Gets the second-order filter input value at time sample n=-2
//! \param[in] handle  The filter handle
//! \return    The input value at time sample n=-2
static inline _iq FILTER_SO_get_x2(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->x2);
} // end of FILTER_SO_get_x2() function


//! \brief     Gets the second-order filter output value at time sample n=-1
//! \param[in] handle  The filter handle
//! \return    The output value at time sample n=-1
static inline _iq FILTER_SO_get_y1(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->y1);
} // end of FILTER_SO_get_y1() function


//! \brief     Gets the second-order filter output value at time sample n=-2
//! \param[in] handle  The filter handle
//! \return    The output value at time sample n=-2
static inline _iq FILTER_SO_get_y2(FILTER_SO_Handle handle)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  return(obj->y2);
} // end of FILTER_SO_get_y2() function


//! \brief     Gets the second-order filter denominator coefficients
//! \param[in] handle  The filter handle
//! \param[in] pa1     The pointer to memory for the filter coefficient value for z^(-1)
//! \param[in] pa2     The pointer to memory for the filter coefficient value for z^(-2)
extern void FILTER_SO_getDenCoeffs(FILTER_SO_Handle handle,_iq *pa1,_iq *pa2);


//! \brief     Gets the initial conditions of the second-order filter
//! \param[in] handle  The filter handle
//! \param[in] px1     The pointer to memory for the input value at time sample n=-1
//! \param[in] px2     The pointer to memory for the input value at time sample n=-2
//! \param[in] py1     The pointer to memory for the output value at time sample n=-1
//! \param[in] py2     The pointer to memory for the output value at time sample n=-2
extern void FILTER_SO_getInitialConditions(FILTER_SO_Handle handle,_iq *px1,_iq *px2,_iq *py1,_iq *py2);


//! \brief     Gets the second-order filter numerator coefficients
//! \param[in] handle  The filter handle
//! \param[in] pb0     The pointer to memory for the filter coefficient value for z^0
//! \param[in] pb1     The pointer to memory for the filter coefficient value for z^(-1)
//! \param[in] pb2     The pointer to memory for the filter coefficient value for z^(-2)
extern void FILTER_SO_getNumCoeffs(FILTER_SO_Handle handle,_iq *pb0,_iq *pb1,_iq *pb2);


//! \brief     Initializes the second-order filter
//! \param[in] pMemory   A pointer to the memory for the second-order filter object
//! \param[in] numBytes  The number of bytes allocated for the second-order filter object, bytes
//! \return The filter (FILTER) object handle
extern FILTER_SO_Handle FILTER_SO_init(void *pMemory,const size_t numBytes);


//! \brief     Runs a second-order filter of the form
//!            y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
//!
//! \param[in] handle      The filter handle
//! \param[in] inputValue  The input value to filter
//! \return The output value from the filter
static inline _iq FILTER_SO_run(FILTER_SO_Handle handle,const _iq inputValue)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  _iq a1 = obj->a1;
  _iq a2 = obj->a2;
  _iq b0 = obj->b0;
  _iq b1 = obj->b1;
  _iq b2 = obj->b2;
  _iq x1 = obj->x1;
  _iq x2 = obj->x2;
  _iq y1 = obj->y1;
  _iq y2 = obj->y2;


  // compute the output
  _iq y0 = _IQmpy(b0,inputValue) + _IQmpy(b1,x1) + _IQmpy(b2,x2) 
    - _IQmpy(a1,y1) - _IQmpy(a2,y2);


  // store values for next time
  obj->x1 = inputValue;
  obj->x2 = x1;
  obj->y1 = y0;
  obj->y2 = y1;

  return(y0);
} // end of FILTER_SO_run() function


//! \brief     Runs a simplified second-order filter of the form
//!            y[n] = b0*x[n] - a1*y[n-1] - a2*y[n-2]
//!
//! \param[in] handle      The filter handle
//! \param[in] inputValue  The input value to filter
//! \return The output value from the filter
static inline _iq FILTER_SO_run_form_0(FILTER_SO_Handle handle,const _iq inputValue)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  _iq a1 = obj->a1;
  _iq a2 = obj->a2;
  _iq b0 = obj->b0;
  _iq y1 = obj->y1;
  _iq y2 = obj->y2;


  // compute the output
  _iq y0 = _IQmpy(b0,inputValue) - _IQmpy(a1,y1) - _IQmpy(a2,y2);


  // store values for next time
  obj->y1 = y0;
  obj->y2 = y1;

  return(y0);
} // end of FILTER_SO_run_form_0() function


//! \brief     Runs a second-order filter of the form
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1] - a2*y[n-2]
//!
//! \param[in] handle      The filter handle
//! \param[in] inputValue  The input value to filter
//! \return The output value from the filter
static inline _iq FILTER_SO_run_form_1(FILTER_SO_Handle handle,const _iq inputValue)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  _iq a1 = obj->a1;
  _iq a2 = obj->a2;
  _iq b0 = obj->b0;
  _iq b1 = obj->b1;
  _iq x1 = obj->x1;
  _iq y1 = obj->y1;
  _iq y2 = obj->y2;


  // compute the output
  _iq y0 = _IQmpy(b0,inputValue) + _IQmpy(b1,x1)
    - _IQmpy(a1,y1) - _IQmpy(a2,y2);


  // store values for next time
  obj->x1 = inputValue;
  obj->y1 = y0;
  obj->y2 = y1;

  return(y0);
} // end of FILTER_SO_run_form_1() function


//! \brief     Sets the second-order filter denominator coefficient a1
//! \param[in] handle  The filter handle
//! \param[in] a1      The filter coefficient value for z^(-1)
static inline void FILTER_SO_set_a1(FILTER_SO_Handle handle,const _iq a1)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->a1 = a1;

  return;
} // end of FILTER_SO_set_a1() function


//! \brief     Sets the second-order filter denominator coefficient a2
//! \param[in] handle  The filter handle
//! \param[in] a2      The filter coefficient value for z^(-2)
static inline void FILTER_SO_set_a2(FILTER_SO_Handle handle,const _iq a2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->a2 = a2;

  return;
} // end of FILTER_SO_set_a2() function


//! \brief     Sets the second-order filter numerator coefficient b0
//! \param[in] handle  The filter handle
//! \param[in] b0      The filter coefficient value for z^0
static inline void FILTER_SO_set_b0(FILTER_SO_Handle handle,const _iq b0)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->b0 = b0;

  return;
} // end of FILTER_SO_set_b0() function


//! \brief     Sets the second-order filter numerator coefficient b1
//! \param[in] handle  The filter handle
//! \param[in] b1      The filter coefficient value for z^(-1)
static inline void FILTER_SO_set_b1(FILTER_SO_Handle handle,const _iq b1)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->b1 = b1;

  return;
} // end of FILTER_SO_set_b1() function


//! \brief     Sets the second-order filter numerator coefficient b2
//! \param[in] handle  The filter handle
//! \param[in] b2      The filter coefficient value for z^(-2)
static inline void FILTER_SO_set_b2(FILTER_SO_Handle handle,const _iq b2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->b2 = b2;

  return;
} // end of FILTER_SO_set_b2() function


//! \brief     Sets the second-order filter input value at time sample n=-1
//! \param[in] handle  The filter handle
//! \param[in] x1      The input value at time sample n=-1
static inline void FILTER_SO_set_x1(FILTER_SO_Handle handle,const _iq x1)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->x1 = x1;

  return;
} // end of FILTER_SO_set_x1() function


//! \brief     Sets the second-order filter input value at time sample n=-2
//! \param[in] handle  The filter handle
//! \param[in] x2      The input value at time sample n=-2
static inline void FILTER_SO_set_x2(FILTER_SO_Handle handle,const _iq x2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->x2 = x2;

  return;
} // end of FILTER_SO_set_x2() function


//! \brief     Sets the second-order filter output value at time sample n=-1
//! \param[in] handle  The filter handle
//! \param[in] y1      The output value at time sample n=-1
static inline void FILTER_SO_set_y1(FILTER_SO_Handle handle,const _iq y1)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->y1 = y1;

  return;
} // end of FILTER_SO_set_y1() function


//! \brief     Sets the second-order filter output value at time sample n=-2
//! \param[in] handle  The filter handle
//! \param[in] y2      The output value at time sample n=-2
static inline void FILTER_SO_set_y2(FILTER_SO_Handle handle,const _iq y2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;

  obj->y2 = y2;

  return;
} // end of FILTER_SO_set_y2() function


//! \brief     Sets the second-order filter denominator coefficients
//! \param[in] handle  The filter handle
//! \param[in] a1      The filter coefficient value for z^(-1)
//! \param[in] a2      The filter coefficient value for z^(-2)
extern void FILTER_SO_setDenCoeffs(FILTER_SO_Handle handle,const _iq a1,const _iq a2);


//! \brief     Sets the initial conditions of the second-order filter
//! \param[in] handle  The filter handle
//! \param[in] x1      The input value at time sample n=-1
//! \param[in] x2      The input value at time sample n=-2
//! \param[in] y1      The output value at time sample n=-1
//! \param[in] y2      The output value at time sample n=-2
extern void FILTER_SO_setInitialConditions(FILTER_SO_Handle handle,const _iq x1,const _iq x2,const _iq y1,const _iq y2);


//! \brief     Sets the second-order filter numerator coefficients
//! \param[in] handle  The filter handle
//! \param[in] b0      The filter coefficient value for z^0
//! \param[in] b1      The filter coefficient value for z^(-1)
//! \param[in] b2      The filter coefficient value for z^(-2)
extern void FILTER_SO_setNumCoeffs(FILTER_SO_Handle handle,const _iq b0,const _iq b1,const _iq b2);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _FILTER_SO_H_ definition


