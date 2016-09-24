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
//! \file   drivers/pll/src/32b/f28x/f2802x/pll.c
//!
//! \brief  Contains the various functions related to the phase-locked loop
//!         (PLL) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "pll.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void PLL_disable(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pll->PLLSTS |= PLL_PLLSTS_PLLOFF_BITS;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_disable() function


void PLL_disableClkDetect(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pll->PLLSTS |= PLL_PLLSTS_MCLKOFF_BITS;
 
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_disableClkDetect() function


void PLL_disableNormRdy(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pll->PLLSTS &= (~PLL_PLLSTS_NORMRDYE_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_disableNormRdy() function


void PLL_disableOsc(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pll->PLLSTS |= PLL_PLLSTS_OSCOFF_BITS;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_disableOsc() function


void PLL_enable(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pll->PLLSTS &= (~PLL_PLLSTS_PLLOFF_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_enable() function


void PLL_enableClkDetect(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pll->PLLSTS &= (~PLL_PLLSTS_MCLKOFF_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_enableClkDetect() function


void PLL_enableNormRdy(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pll->PLLSTS |= (uint16_t)PLL_PLLSTS_NORMRDYE_BITS;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_enableNormRdy() function


void PLL_enableOsc(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pll->PLLSTS &= (~PLL_PLLSTS_OSCOFF_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_enableOsc() function


PLL_ClkFreq_e PLL_getClkFreq(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    // get the clock rate
  PLL_ClkFreq_e clkFreq = (PLL_ClkFreq_e)(pll->PLLCR & PLL_PLLCR_DIV_BITS);

  return(clkFreq);
} // end of PLL_getClkFreq() function


PLL_ClkStatus_e PLL_getClkStatus(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;

    // mask the bits
    PLL_ClkStatus_e status = (PLL_ClkStatus_e)(pll->PLLSTS & PLL_PLLSTS_MCLKSTS_BITS);

    return(status);
} // end of PLL_getClkStatus() function


PLL_DivideSelect_e PLL_getDivideSelect(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;

    // mask the bits
    PLL_DivideSelect_e divSelect = (PLL_DivideSelect_e)(pll->PLLSTS & PLL_PLLSTS_DIVSEL_BITS);

    return(divSelect);
} // end of PLL_getDivideSelect() function


PLL_LockStatus_e PLL_getLockStatus(PLL_Handle pllHandle)
{
    volatile PLL_Obj *pll = (PLL_Obj *)pllHandle;


    // mask the bits
    PLL_LockStatus_e status = (PLL_LockStatus_e)(pll->PLLSTS & PLL_PLLSTS_PLLLOCKS_BITS);

    return(status);
} // end of PLL_getLockStatus() function


PLL_Handle PLL_init(void *pMemory, const size_t numBytes)
{
    PLL_Handle pllHandle;


    if(numBytes < sizeof(PLL_Obj))
    return((PLL_Handle)NULL);

    // assign the handle
    pllHandle = (PLL_Handle)pMemory;

    return(pllHandle);
} // end of PLL_init() function


void PLL_resetClkDetect(PLL_Handle pllHandle)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pll->PLLSTS |= PLL_PLLSTS_MCLKCLR_BITS;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_resetClkDetect() function


void PLL_setClkFreq(PLL_Handle pllHandle,const PLL_ClkFreq_e clkFreq)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
  pll->PLLCR = clkFreq;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_setClkFreq() function


void PLL_setDivideSelect(PLL_Handle pllHandle,const PLL_DivideSelect_e divSelect)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pll->PLLSTS &= (~PLL_PLLSTS_DIVSEL_BITS);

    // set the bits
    pll->PLLSTS |= divSelect;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_setDivideSelect() function


void PLL_setLockPeriod(PLL_Handle pllHandle, const uint16_t lockPeriod)
{
    PLL_Obj *pll = (PLL_Obj *)pllHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pll->PLLLOCKPRD = lockPeriod;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PLL_setLockPeriod() function


// end of file
