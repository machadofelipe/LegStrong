/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
//! \file   drivers/osc/src/32b/f28x/f2802x/osc.c
//! \brief  Contains the various functions related to the 
//!         oscillator object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "osc.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


OSC_Handle OSC_init(void *pMemory,const size_t numBytes)
{
  OSC_Handle oscHandle;


  if(numBytes < sizeof(OSC_Obj))
    return((OSC_Handle)NULL);

  // assign the handle
  oscHandle = (OSC_Handle)pMemory;

  return(oscHandle);
} // end of OSC_init() function


void OSC_setCoarseTrim(OSC_Handle clkHandle,
                       const OSC_Number_e oscNumber,
                       const uint_least8_t trimValue)
{
  OSC_Obj *osc = (OSC_Obj *)clkHandle;

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(oscNumber == OSC_Number_1)
    {
      // clear the bits
      osc->INTOSC1TRIM &= (~OSC_INTOSCnTRIM_COARSE_BITS);

      // set the bits
      osc->INTOSC1TRIM |= trimValue;
    }
  else
    {
      // clear the bits
      osc->INTOSC2TRIM &= (~OSC_INTOSCnTRIM_COARSE_BITS);

      // set the bits
      osc->INTOSC2TRIM |= trimValue;
    }
    
  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of OSC_setCoarseTrim() function


void OSC_setFineTrim(OSC_Handle clkHandle,
                     const OSC_Number_e oscNumber,
                     const uint_least8_t trimValue)
{
  OSC_Obj *osc = (OSC_Obj *)clkHandle;

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(oscNumber == OSC_Number_1)
    {
      // clear the bits
      osc->INTOSC1TRIM &= (~OSC_INTOSCnTRIM_FINE_BITS);

      // set the bits
      osc->INTOSC1TRIM |= trimValue << 9;
    }
  else
    {
      // clear the bits
      osc->INTOSC2TRIM &= (~OSC_INTOSCnTRIM_FINE_BITS);

      // set the bits
      osc->INTOSC2TRIM |= trimValue << 9;
    }
    
  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of OSC_setFineTrim() function


void OSC_setTrim(OSC_Handle clkHandle,
                     const OSC_Number_e oscNumber,
                     const uint16_t trimValue)
{
  OSC_Obj *osc = (OSC_Obj *)clkHandle;

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(oscNumber == OSC_Number_1)
    {
      // set the bits
      osc->INTOSC1TRIM = trimValue;
    }
  else
    {
      // set the bits
      osc->INTOSC2TRIM = trimValue;
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of OSC_setTrim() function


// end of file
