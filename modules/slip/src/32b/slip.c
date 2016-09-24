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
//! \file   modules/slip/src/32b/slip.c
//! \brief  Portable C fixed point code.  These functions define the
//! \brief  slip compensation routines.
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "slip.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

SLIP_Handle SLIP_init(void *pMemory,const size_t numBytes)
{
  SLIP_Handle slipHandle;
  
  if(numBytes < sizeof(SLIP_Obj))
    return((SLIP_Handle)NULL);

  // assign the handle
  slipHandle = (SLIP_Handle)pMemory;
  
  return(slipHandle);
} // end of SLIP_init() function


void SLIP_setup(SLIP_Handle slipHandle, _iq sampleTime)
{
  SLIP_Obj *slip;
 
  // create an object pointer for manipulation
  slip = (SLIP_Obj *) slipHandle;
  
  // set the sample time
  slip->sample_time = sampleTime;

  // initialize all other values to 0
  slip->enc_elec_angle = 0;
  slip->enc_magnetic_angle = 0;
  slip->enc_slip_angle = 0;
  slip->incremental_slip = 0;
  
  return;
} // end of SLIP_setup() function


void SLIP_run(SLIP_Handle slipHandle)
{
  SLIP_Obj *slip;
  uint32_t temp;
	
  // create an object pointer for manipulation
  slip = (SLIP_Obj *) slipHandle;

  // update the slip angle
  slip->enc_slip_angle = slip->enc_slip_angle + slip->incremental_slip;
  // wrap around 1.0 (Q24)
  slip->enc_slip_angle &= ((uint32_t) 0x00ffffff);
  // add in compensation for slip
  temp = slip->enc_elec_angle + slip->enc_slip_angle;
  // wrap around 1.0 (Q24)
  temp &= ((uint32_t) 0x00ffffff);
  // store encoder magnetic angle
  slip->enc_magnetic_angle = (_iq)temp;
  
  return;
} // end of SLIP_run() function


// end of file
