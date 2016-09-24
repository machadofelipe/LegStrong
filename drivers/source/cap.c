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
//! \file   drivers/cap/src/32b/f28x/f2802x/cap.c
//! \brief  Contains the various functions related to the 
//!         Enhanced Capture (CAP) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include "cap.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


CAP_Handle CAP_init(void *pMemory, const size_t numBytes)
{
    CAP_Handle capHandle;


    if(numBytes < sizeof(CAP_Obj))
    return((CAP_Handle)NULL);

    // assign the handle
    capHandle = (CAP_Handle)pMemory;
    
    return(capHandle);
} // end of CAP_init() function

void CAP_disableCaptureLoad(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;

    // clear the bits
    cap->ECCTL1 &= ~CAP_ECCTL1_CAPLDEN_BITS;

    return;
} // end of CAP_disableCaptureLoad() function

void CAP_disableSyncIn(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // set the bit
    cap->ECCTL2 &= ~CAP_ECCTL2_SYNCIEN_BITS;

    return;
} // end of CAP_disableSyncIn() function

void CAP_disableInt(CAP_Handle capHandle, const CAP_Int_Type_e intType)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECEINT &= ~intType;

    return;
} // end of CAP_disableInt() function

void CAP_disableTimestampCounter(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;

    // clear the bits
    cap->ECCTL2 &= ~CAP_ECCTL2_TSCTRSTOP_BITS;

    return;
} // end of CAP_disableTimestampCounter() function

void CAP_enableCaptureLoad(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;

    // clear the bits
    cap->ECCTL1 |= CAP_ECCTL1_CAPLDEN_BITS;

    return;
} // end of CAP_enableCaptureLoad() function

void CAP_enableInt(CAP_Handle capHandle, const CAP_Int_Type_e intType)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECEINT |= intType;

    return;
} // end of CAP_enableInt() function

void CAP_enableSyncIn(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // set the bit
    cap->ECCTL2 |= CAP_ECCTL2_SYNCIEN_BITS;

    return;
} // end of CAP_enableSyncIn() function

void CAP_enableTimestampCounter(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;

    // clear the bits
    cap->ECCTL2 |= CAP_ECCTL2_TSCTRSTOP_BITS;

    return;
} // end of CAP_enableTimestampCounter() function

void CAP_setCapEvtPolarity(CAP_Handle capHandle, const CAP_Event_e event, const CAP_Polarity_e polarity)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;

    // clear the bits
    cap->ECCTL1 &= ~(1 << (2 * event));
    
    // Set the new value
    cap->ECCTL1 |= (polarity << (2 * event));

    return;
} // end of CAP_setCapEvtPolarity() function

void CAP_setCapEvtReset(CAP_Handle capHandle, const CAP_Event_e event, const CAP_Reset_e reset)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;

    // clear the bits
    cap->ECCTL1 &= ~(1 << (1 + (2 * event)));
    
    // Set the new value
    cap->ECCTL1 |= (reset << (1 + (2 * event)));

    return;
} // end of CAP_setCapEvtReset() function

void CAP_setCapContinuous(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECCTL2 &= (~CAP_ECCTL2_CONTONESHOT_BITS);

    return;
} // end of CAP_setCapContinuous() function

void CAP_setCapOneShot(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECCTL2 |= CAP_ECCTL2_CONTONESHOT_BITS;

    return;
} // end of CAP_setCapOneShot() function

void CAP_setModeCap(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECCTL2 &= (~CAP_ECCTL2_CAPAPWM_BITS);

    return;
} // end of CAP_setModeCap() function

void CAP_setModeApwm(CAP_Handle capHandle)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECCTL2 |= CAP_ECCTL2_CAPAPWM_BITS;

    return;
} // end of CAP_setModeApwm() function

void CAP_setStopWrap(CAP_Handle capHandle, const CAP_Stop_Wrap_e stopWrap)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECCTL2 &= (~CAP_ECCTL2_STOP_WRAP_BITS);
    
    // Set the new value
    cap->ECCTL2 |= stopWrap;

    return;
} // end of CAP_setStopWrap() function

void CAP_setSyncOut(CAP_Handle capHandle, const CAP_SyncOut_e syncOut)
{
    CAP_Obj *cap = (CAP_Obj *)capHandle;


    // clear the bits
    cap->ECCTL2 &= (~CAP_ECCTL2_SYNCOSEL_BITS);
    
    // Set the new value
    cap->ECCTL2 |= syncOut;

    return;
} // end of CAP_setSyncOut() function

// end of file
