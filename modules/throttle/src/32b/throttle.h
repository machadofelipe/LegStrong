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
#ifndef _THROTTLE_H_
#define _THROTTLE_H_

//! \file   ~sw/modules/throttle/src/float/throttle.h
//! \brief  Contains the public interface to the 
//!         math (MATH) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"
#include "math.h"


//!
//!
//! \defgroup THROTTLE THROTTLE
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup THROTTLE_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the throttle state machine
//!
typedef enum
{
  THROTTLE_CalMaxMin = 0,  
  THROTTLE_CalCalc = 1,
  THROTTLE_Run = 2
} THROTTLE_States_e;




//! \brief      Defines the encoder object
//! \details    The encoder object takes in data from a QEP peripheral and calculates speed
//!             and position.
//!
typedef struct _THROTTLE_Obj_
{
  _iq                 max_adc;                  //!<Maximum value that is measured from the throttle.
  _iq                 min_adc;                  //!<Minimum value that is measured from the throttle.
  _iq                 max_out;                  //!<Maximum value that is output as a result.
  _iq                 min_out;                  //!<Minimum value that is output as a result.
  _iq                 slope;                    //!<Slope of y=mx+b of the throttle.
  _iq                 offset;                   //!<Offset of y=mx+b of the throttle.
  _iq                 value;                    //!<The input value (x) of the throttle.
  _iq                 result;                   //!<The output corrected result of the throttle module.
  bool                flagSw1;                  //!<Switch to put the throttle into max and min calibration mode.
  bool                flagSw2;                  //!<Switch to take the throttle out of max and min calibration mode.
  THROTTLE_States_e   state;                    //!<The state of the throttle.
} THROTTLE_Obj;


//! \brief      Defines the THROTTLE handle
//! \details    The THROTTLE handle is a pointer to a THROTTLE object.  In all THROTTLE functions
//!             the THROTTLE handle is passed so that the function knows what peripherals
//!             are to be accessed.
//!
typedef struct _THROTTLE_Obj_ *THROTTLE_Handle;


// **************************************************************************
// the function prototypes


//! \brief     Initializes the throttle module
//! \param[in] pMemory      A pointer to the memory for the throttle object
//! \param[in] numBytes     The number of bytes allocated for the throttle, bytes
//! \return                 The encoder (THROTTLE) object handle
extern THROTTLE_Handle THROTTLE_init(void *pMemory, const size_t numBytes);


//! \brief     Sets up the throttle module parameters initially
//! \param[in] handle  The throttle handle
//! \param[in] handle  The maximum adc measured value in per unit
//! \param[in] handle  The minimum adc measured value in per unit
//! \param[in] handle  The maximum output result in per unit
//! \param[in] handle  The minimum output result in per unit
extern void THROTTLE_setParams(THROTTLE_Handle handle,  \
                                const bool invert,      \
                                const _iq max_adc,      \
                                const _iq min_adc,      \
                                const _iq max_out,      \
                                const _iq min_out);


extern void THROTTLE_setup(THROTTLE_Handle handle,      \
                                const _iq value,        \
                                const bool SW1,         \
                                const bool SW2);

                                
extern void THROTTLE_runState(THROTTLE_Handle handle);


static inline _iq THROTTLE_getResult(THROTTLE_Handle handle)
{
  THROTTLE_Obj *obj = (THROTTLE_Obj *)handle;

  return obj->result;
}


#endif
