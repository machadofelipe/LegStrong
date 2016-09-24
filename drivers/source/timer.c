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
//! \file   drivers/timer/src/32b/f28x/f2802x/timer.c
//!
//! \brief  Contains the various functions related to the timer (TIMER) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

#include "timer.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void TIMER_clearFlag(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // set the bits
    timer->TCR |= (uint16_t)TIMER_TCR_TIF_BITS;

    return;
} // end of TIMER_clearFlag() function


void TIMER_disableInt(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // clear the bits
    timer->TCR &= (~(uint16_t)TIMER_TCR_TIE_BITS);

    return;
} // end of TIMER_disableInt() function


void TIMER_enableInt(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // set the bits
    timer->TCR |= (uint16_t)TIMER_TCR_TIE_BITS;

    return;
} // end of TIMER_enableInt() function


TIMER_Status_e TIMER_getStatus(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // get the status
    TIMER_Status_e status = (TIMER_Status_e)(timer->TCR & (uint16_t)TIMER_TCR_TIF_BITS);

    return(status);
} // end of TIMER_getStatus() function


TIMER_Handle TIMER_init(void *pMemory, const size_t numBytes)
{
    TIMER_Handle timerHandle;


    if(numBytes < sizeof(TIMER_Obj))
    return((TIMER_Handle)NULL);

    // assign the handle
    timerHandle = (TIMER_Handle)pMemory;

    return(timerHandle);
} // end of TIMER_init() function


void TIMER_setDecimationFactor(TIMER_Handle timerHandle, 
                               const uint16_t decFactor)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // set the bits
    timer->TPR =
    ((uint32_t)(decFactor & 0xFF00) << 8) | (uint32_t)(decFactor & 0x00FF);

    return;
} // end of TIMER_setDecimationFactor() function


void TIMER_setEmulationMode(TIMER_Handle timerHandle, 
                            const TIMER_EmulationMode_e mode)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;

    
    // clear the bits
    timer->TCR &= (~(uint16_t)TIMER_TCR_FREESOFT_BITS);

    // set the bits
    timer->TCR |= (uint16_t)mode;

    return;
} // end of TIMER_setEmulationMode() function


void TIMER_setPreScaler(TIMER_Handle timerHandle, 
                        const uint16_t preScaler)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // set the bits
    timer->TPR =
    ((uint32_t)(preScaler & 0xFF00) << 8) | (uint32_t)(preScaler & 0x00FF);

    return;
} // end of TIMER_setPreScaler() function


// end of file
