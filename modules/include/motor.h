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
#ifndef _MOTOR_H_
#define _MOTOR_H_

//! \file   modules/motor/src/motor.h
//! \brief  Contains motor related definitions
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"


//!
//!
//! \defgroup MOTOR MOTOR
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup MOTOR_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//! \brief Enumeration for the motor types
//!
typedef enum 
{
  MOTOR_Type_Induction=0,     //!< induction
  MOTOR_Type_Pm               //!< permanent magnet
} MOTOR_Type_e;


//! \brief Defines the motor parameters
//!
typedef struct _MOTOR_Params_
{
  MOTOR_Type_e    type;               //!< Defines the motor type

  uint_least16_t  numPolePairs;       //!< Defines the number of pole pairs

  float_t         Lr_H;               //!< Defines the rotor inductance, H

  float_t         Ls_d_H;             //!< Defines the direct stator inductance, H
  float_t         Ls_q_H;             //!< Defines the quadrature stator inductance, H

  float_t         Rr_Ohm;             //!< Defines the rotor resistance, Ohm

  float_t         Rs_Ohm;             //!< Defines the stator resistance, Ohm
  
  float_t         ratedFlux_VpHz;     //!< Defines the rated flux, V/Hz
} MOTOR_Params;


// **************************************************************************
// the functions


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _MOTOR_H_ definition





