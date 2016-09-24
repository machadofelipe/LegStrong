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

//! \file   drivers/wdog/src/32b/f28x/f2802x/wdog.c
//! \brief  Contains the various functions related to the 
//!         watch dog (WDOG) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "wdog.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void WDOG_clearCounter(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // write first sequence
    wdog->WDKEY = 0x55;

    // write second sequence
    wdog->WDKEY = 0xAA;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_clearCounter() function


void WDOG_disable(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;
    uint16_t regValue = wdog->WDCR;


    // set the bits
    regValue |= WDOG_WDCR_WDDIS_BITS;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // store the result
    wdog->WDCR = regValue | WDOG_WDCR_WRITE_ENABLE;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_disable() function


void WDOG_disableInt(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    wdog->SCSR &= (~WDOG_SCSR_WDENINT_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_disableInt() function


void WDOG_disableOverRide(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    wdog->SCSR &= (~WDOG_SCSR_WDOVERRIDE_BITS);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_disableOverRide() function


void WDOG_enable(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;
    uint16_t regValue = wdog->WDCR;


    // clear the bits
    regValue &= (~WDOG_WDCR_WDDIS_BITS);

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // store the result
    wdog->WDCR = regValue | WDOG_WDCR_WRITE_ENABLE;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_enable() function


void WDOG_enableInt(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    wdog->SCSR |= WDOG_SCSR_WDENINT_BITS;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_enableInt() function


void WDOG_enableOverRide(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    wdog->SCSR |= WDOG_SCSR_WDOVERRIDE_BITS;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_enableOverRide() function


WDOG_IntStatus_e WDOG_getIntStatus(WDOG_Handle wdogHandle)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;

    // mask the bits
    WDOG_IntStatus_e status = 
    (WDOG_IntStatus_e)(wdog->SCSR & WDOG_SCSR_WDINTS_BITS);

    return(status);
} // end of WDOG_getIntStatus() function


WDOG_Handle WDOG_init(void *pMemory, const size_t numBytes)
{
    WDOG_Handle wdogHandle;


    if(numBytes < sizeof(WDOG_Obj))
    return((WDOG_Handle)NULL);

    // assign the handle
    wdogHandle = (WDOG_Handle)pMemory;

    return(wdogHandle);
} // end of WDOG_init() function


void WDOG_setCount(WDOG_Handle wdogHandle, const uint_least8_t count)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;


    // set the bits
    wdog->WDCNTR = count;

    return;
} // end of WDOG_setCount() function


void WDOG_setPreScaler(WDOG_Handle wdogHandle, const WDOG_PreScaler_e preScaler)
{
    WDOG_Obj *wdog = (WDOG_Obj *)wdogHandle;
    uint16_t regValue = wdog->WDCR;


    // clear the bits
    regValue &= (~WDOG_WDCR_WDPS_BITS);

    // set the bits
    regValue |= preScaler;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // store the result
    wdog->WDCR = regValue | WDOG_WDCR_WRITE_ENABLE;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of WDOG_setPreScaler() function

// end of file
