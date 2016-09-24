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
//! \file   modules/offset/src/32b/offset.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         offset (OFFSET) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "offset.h"



// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


_iq OFFSET_getBeta(OFFSET_Handle handle)
{
  OFFSET_Obj *obj = (OFFSET_Obj *)handle;
  _iq b0;
  _iq b1;

  FILTER_FO_getNumCoeffs(obj->filterHandle,&b0,&b1);

  return(b0);
} // end of OFFSET_getBeta() function


OFFSET_Handle OFFSET_init(void *pMemory,const size_t numBytes)
{
  OFFSET_Handle handle;
  OFFSET_Obj *obj;

  if(numBytes < sizeof(OFFSET_Obj))
    return((OFFSET_Handle)NULL);

  // assign the handle
  handle = (OFFSET_Handle)pMemory;
  obj = (OFFSET_Obj *)handle;

  obj->filterHandle = FILTER_FO_init(&(obj->filter),sizeof(obj->filter));

  return(handle);
} // end of OFFSET_init() function


void OFFSET_setBeta(OFFSET_Handle handle,const _iq beta)
{
  OFFSET_Obj *obj = (OFFSET_Obj *)handle;
  _iq a1 = (beta - _IQ(1.0));
  _iq b0 = beta;
  _iq b1 = 0;

  FILTER_FO_setDenCoeffs(obj->filterHandle,a1);
  FILTER_FO_setNumCoeffs(obj->filterHandle,b0,b1);

  return;
} // end of OFFSET_setBeta() function


void OFFSET_setInitCond(OFFSET_Handle handle,const _iq initCond)
{
  OFFSET_Obj *obj = (OFFSET_Obj *)handle;

  FILTER_FO_setInitialConditions(obj->filterHandle,initCond,initCond);
  obj->value = initCond;

  return;
} // end of OFFSET_setInitCond() function


void OFFSET_setOffset(OFFSET_Handle handle, _iq offsetValue)
{
  OFFSET_Obj *obj = (OFFSET_Obj *)handle;

  obj->value = offsetValue;

  return;
} // end of OFFSET_setOffset() function

// end of file











