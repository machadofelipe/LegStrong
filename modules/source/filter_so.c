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
//! \file   modules/filter/src/32b/filter_so.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         second-order filter (FILTER) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "filter_so.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


void FILTER_SO_getDenCoeffs(FILTER_SO_Handle handle,_iq *pa1,_iq *pa2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;


  *pa1 = obj->a1;
  *pa2 = obj->a2;

  return;
} // end of FILTER_SO_getDenCoeffs() function


void FILTER_SO_getInitialConditions(FILTER_SO_Handle handle,_iq *px1,_iq *px2,
                                    _iq *py1,_iq *py2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;


  *px1 = obj->x1;
  *px2 = obj->x2;

  *py1 = obj->y1;
  *py2 = obj->y2;

  return;
} // end of FILTER_SO_getInitialConditions() function


void FILTER_SO_getNumCoeffs(FILTER_SO_Handle handle,_iq *pb0,_iq *pb1,_iq *pb2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;


  *pb0 = obj->b0;
  *pb1 = obj->b1;
  *pb2 = obj->b2;

  return;
} // end of FILTER_SO_getNumCoeffs() function


FILTER_SO_Handle FILTER_SO_init(void *pMemory,const size_t numBytes)
{
  FILTER_SO_Handle handle;


  if(numBytes < sizeof(FILTER_SO_Obj))
    return((FILTER_SO_Handle)NULL);

  // assign the handle
  handle = (FILTER_SO_Handle)pMemory;

  return(handle);
} // end of FILTER_SO_init() function


void FILTER_SO_setDenCoeffs(FILTER_SO_Handle handle,const _iq a1,const _iq a2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;


  obj->a1 = a1;
  obj->a2 = a2;

  return;
} // end of FILTER_SO_setDenCoeffs() function


void FILTER_SO_setInitialConditions(FILTER_SO_Handle handle,const _iq x1,const _iq x2,
                                    const _iq y1,const _iq y2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;


  obj->x1 = x1;
  obj->x2 = x2;

  obj->y1 = y1;
  obj->y2 = y2;

  return;
} // end of FILTER_SO_setInitialConditions() function


void FILTER_SO_setNumCoeffs(FILTER_SO_Handle handle,const _iq b0,const _iq b1,const _iq b2)
{
  FILTER_SO_Obj *obj = (FILTER_SO_Obj *)handle;


  obj->b0 = b0;
  obj->b1 = b1;
  obj->b2 = b2;

  return;
} // end of FILTER_SO_setNumCoeffs() function

// end of file




