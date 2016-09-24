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
//! \file   drivers/flash/src/32b/f28x/f2802x/flash.c
//! \brief  Contains the various functions related to the 
//!         flash object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "flash.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void FLASH_clear3VStatus(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  flash->FSTATUS |= FLASH_FSTATUS_3VSTAT_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_clear3VStatus() function


#pragma CODE_SECTION(FLASH_disablePipelineMode, "ramfuncs");
void FLASH_disablePipelineMode(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  flash->FOPT &= (~FLASH_FOPT_ENPIPE_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_disablePipelineMode() function


#pragma CODE_SECTION(FLASH_enablePipelineMode, "ramfuncs");
void FLASH_enablePipelineMode(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  flash->FOPT |= FLASH_FOPT_ENPIPE_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_enablePipelineMode() function


FLASH_3VStatus_e FLASH_get3VStatus(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  // get the status
  FLASH_3VStatus_e status = (FLASH_3VStatus_e)(flash->FSTATUS & FLASH_FSTATUS_3VSTAT_BITS);

  return(status);
} // end of FLASH_get3VStatus() function


uint16_t FLASH_getActiveWaitCount(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  // get the status
  uint16_t count = (flash->FACTIVEWAIT & FLASH_FACTIVEWAIT_ACTIVEWAIT_BITS);

  return(count);
} // end of FLASH_getActiveWaitCount() function


FLASH_CounterStatus_e FLASH_getActiveWaitStatus(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  // get the status
  FLASH_CounterStatus_e status = (FLASH_CounterStatus_e)((flash->FSTATUS & FLASH_FSTATUS_ACTIVEWAITS_BITS) >> 3);

  return(status);
} // end of FLASH_getActiveWaitStatus() function


FLASH_PowerMode_e FLASH_getPowerMode(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  // get the bits
  FLASH_PowerMode_e mode = (FLASH_PowerMode_e)(flash->FSTATUS & FLASH_FSTATUS_PWRS_BITS);

  return(mode);
} // end of FLASH_getPowerMode() function


uint16_t FLASH_getStandbyWaitCount(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  // get the status
  uint16_t count = (flash->FSTDBYWAIT & FLASH_FSTDBYWAIT_STDBYWAIT_BITS);

  return(count);
} // end of FLASH_getStandbyWaitCount() function


FLASH_CounterStatus_e FLASH_getStandbyWaitStatus(FLASH_Handle flashHandle)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  // get the status
  FLASH_CounterStatus_e status = (FLASH_CounterStatus_e)((flash->FSTATUS & FLASH_FSTATUS_STDBYWAITS_BITS) >> 2);

  return(status);
} // end of FLASH_getStandbyWaitStatus() function


FLASH_Handle FLASH_init(void *pMemory,const size_t numBytes)
{
  FLASH_Handle flashHandle;


  if(numBytes < sizeof(FLASH_Obj))
    return((FLASH_Handle)NULL);

  // assign the handle
  flashHandle = (FLASH_Handle)pMemory;

  return(flashHandle);
} // end of FLASH_init() function


#pragma CODE_SECTION(FLASH_setActiveWaitCount, "ramfuncs");
void FLASH_setActiveWaitCount(FLASH_Handle flashHandle,const uint16_t count)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  flash->FACTIVEWAIT = count;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_setActiveWaitCount() function


#pragma CODE_SECTION(FLASH_setNumPagedReadWaitStates, "ramfuncs");
void FLASH_setNumPagedReadWaitStates(FLASH_Handle flashHandle,const FLASH_NumPagedWaitStates_e numStates)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  flash->FBANKWAIT &= (~FLASH_FBANKWAIT_PAGEWAIT_BITS);

  // set the bits
  flash->FBANKWAIT |= numStates;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_setNumPagedReadWaitStates() function


#pragma CODE_SECTION(FLASH_setNumRandomReadWaitStates, "ramfuncs");
void FLASH_setNumRandomReadWaitStates(FLASH_Handle flashHandle,const FLASH_NumRandomWaitStates_e numStates)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  flash->FBANKWAIT &= (~FLASH_FBANKWAIT_RANDWAIT_BITS);

  // set the bits
  flash->FBANKWAIT |= numStates;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_setNumRandomReadWaitStates() function


#pragma CODE_SECTION(FLASH_setOtpWaitStates, "ramfuncs");
void FLASH_setOtpWaitStates(FLASH_Handle flashHandle,const FLASH_NumOtpWaitStates_e numStates)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  flash->FOTPWAIT &= (~FLASH_FOTPWAIT_OTPWAIT_BITS);

  // set the bits
  flash->FOTPWAIT |= numStates;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_setOneTimeProgrammableStates() function


#pragma CODE_SECTION(FLASH_setPowerMode, "ramfuncs");
void FLASH_setPowerMode(FLASH_Handle flashHandle,const FLASH_PowerMode_e mode)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  flash->FPWR &= (~FLASH_FPWR_PWR_BITS);

  // set the bits
  flash->FPWR |= mode;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_setPowerMode() function


#pragma CODE_SECTION(FLASH_setStandbyWaitCount, "ramfuncs");
void FLASH_setStandbyWaitCount(FLASH_Handle flashHandle,const uint16_t count)
{
  FLASH_Obj *flash = (FLASH_Obj *)flashHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  flash->FSTDBYWAIT = count;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of FLASH_setStandbyWaitCount() function


// end of file
