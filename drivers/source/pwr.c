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
//! \file   drivers/pwr/src/32b/f28x/f2802x/pwr.c
//!
//! \brief  Contains the various functions related to the power (PWR) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "pwr.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


void PWR_disableBrownOutReset(PWR_Handle pwrHandle)
{
    PWR_Obj *pwr = (PWR_Obj *)pwrHandle;


    // set the bits
    pwr->BORCFG |= PWR_BORCFG_BORENZ_BITS;

    return;
} // end of PWR_disableBrownOutReset() function


void PWR_disableWatchDogInt(PWR_Handle pwrHandle)
{
    PWR_Obj *pwr = (PWR_Obj *)pwrHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pwr->LPMCR0 &= (~PWR_LPMCR0_WDINTE_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PWR_disableWatchDogInt() function


void PWR_enableBrownOutReset(PWR_Handle pwrHandle)
{
    PWR_Obj *pwr = (PWR_Obj *)pwrHandle;


    // clear the bits
    pwr->BORCFG &= (~PWR_BORCFG_BORENZ_BITS);

    return;
} // end of PWR_enableBrownOutReset() function


void PWR_enableWatchDogInt(PWR_Handle pwrHandle)
{
    PWR_Obj *pwr = (PWR_Obj *)pwrHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    pwr->LPMCR0 |= (uint16_t)(PWR_LPMCR0_WDINTE_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PWR_enableWatchDogInt() function


PWR_Handle PWR_init(void *pMemory, const size_t numBytes)
{
    PWR_Handle pwrHandle;


    if(numBytes < sizeof(PWR_Obj))
    return((PWR_Handle)NULL);

    // assign the handle
    pwrHandle = (PWR_Handle)pMemory;

    return(pwrHandle);
} // end of PWR_init() function


void PWR_setLowPowerMode(PWR_Handle pwrHandle, const PWR_LowPowerMode_e lowPowerMode)
{
    PWR_Obj *pwr = (PWR_Obj *)pwrHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    // clear the bits
    pwr->LPMCR0 &= (~PWR_LPMCR0_LPM_BITS);

    // set the bits
    pwr->LPMCR0 |= lowPowerMode;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PWR_enableBrownOutReset() function


void PWR_setNumStandByClocks(PWR_Handle pwrHandle, const PWR_NumStandByClocks_e numClkCycles)
{
    PWR_Obj *pwr = (PWR_Obj *)pwrHandle;
    

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    pwr->LPMCR0 &= (~PWR_LPMCR0_QUALSTDBY_BITS);

    // set the bits
    pwr->LPMCR0 |= numClkCycles;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PWR_setNumStandByClocks() function


// end of file
