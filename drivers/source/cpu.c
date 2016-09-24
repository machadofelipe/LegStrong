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
//! \file   drivers/cpu/src/32b/f28x/f2802x/cpu.c
//! \brief  Contains the various functions related to the 
//!         central processing unit (CPU) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "cpu.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals

CPU_Obj cpu;


// **************************************************************************
// the functions

void CPU_clearIntFlags(CPU_Handle cpuHandle)
{

  // clear the bits
  IFR = 0;

  return;
} // end of CPU_clearIntFlags() function


void CPU_disableDebugInt(CPU_Handle cpuHandle)
{

  // set the bit
  asm(" setc DBGM");

  return;
} // end of CPU_disableDebug() function


void CPU_disableGlobalInts(CPU_Handle cpuHandle)
{

  // set the bit
  asm(" setc INTM");

  return;
} // end of CPU_disableGlobalInts() function


void CPU_disableInt(CPU_Handle cpuHandle,const CPU_IntNumber_e intNumber)
{

  // clear the bit
  IER &= (~intNumber);

  return;
} // end of CPU_disableInt() function


void CPU_disableInts(CPU_Handle cpuHandle)
{

  // clear the bits
  IER = 0;

  return;
} // end of CPU_disableInts() function


void CPU_disableProtectedRegisterWrite(CPU_Handle cpuHandle)
{

  // clear the bits
  asm(" EDIS");

  return;
} // end fo CPU_disableProtectedRegisterWrite() function


void CPU_enableDebugInt(CPU_Handle cpuHandle)
{

  // clear the bit
  asm(" clrc DBGM");

  return;
} // end of CPU_enableDebugInt() function


void CPU_enableGlobalInts(CPU_Handle cpuHandle)
{

  // clear the bit
  asm(" clrc INTM");

  return;
} // end of CPU_enableGlobalInts() function


void CPU_enableInt(CPU_Handle cpuHandle,const CPU_IntNumber_e intNumber)
{

  // set the interrupt 
  IER |= intNumber;

  return;
} // end of CPU_enableInt() function


void CPU_enableProtectedRegisterWrite(CPU_Handle cpuHandle)
{

  // set the bits
  asm(" EALLOW");

  return;
} // end fo CPU_enableProtectedRegisterWrite() function


CPU_Handle CPU_init(void *pMemory,const size_t numBytes)
{
  CPU_Handle cpuHandle;


  if(numBytes < sizeof(CPU_Obj))
    return((CPU_Handle)NULL);

  // assign the handle
  cpuHandle = (CPU_Handle)pMemory;

  return(cpuHandle);
} // end of CPU_init() function


// end of file
