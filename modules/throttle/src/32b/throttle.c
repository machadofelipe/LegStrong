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
 
 
// **************************************************************************
// the includes

// drivers

// modules

// platforms
#include "throttle.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


extern THROTTLE_Handle THROTTLE_init(void *pMemory, const size_t numBytes)
{
  THROTTLE_Handle handle;

  if(numBytes < sizeof(THROTTLE_Obj))
    return((THROTTLE_Handle)NULL);

  // assign the handle
  handle = (THROTTLE_Handle)pMemory;

  return(handle);
}


//! \brief     Sets up the throttle module parameters initially
//! \param[in] handle  The throttle handle
extern void THROTTLE_setParams(THROTTLE_Handle handle,  \
                                const bool invert,
                                const _iq max_adc,      \
                                const _iq min_adc,      \
                                const _iq max_out,      \
                                const _iq min_out)
{
  THROTTLE_Obj *obj = (THROTTLE_Obj *)handle;

  obj->flagSw1 = false;
  obj->flagSw2 = false;
  obj->max_adc = max_adc;
  obj->min_adc = min_adc;

  obj->max_out = max_out;
  obj->min_out = min_out;

  obj->slope = _IQdiv((obj->max_out - obj->min_out),(obj->max_adc - obj->min_adc));
  obj->offset = obj->max_out - _IQmpy(obj->slope,obj->max_adc);

  obj->state = THROTTLE_Run;
  obj->value = _IQ(0.0);

  return;
}


extern void THROTTLE_setup(THROTTLE_Handle handle,      \
                                const _iq value,        \
                                const bool SW1,         \
                                const bool SW2)
{
  THROTTLE_Obj *obj = (THROTTLE_Obj *)handle;

  obj->flagSw1 = SW1;
  obj->flagSw2 = SW2;

  obj->value = value;

  return;
}

                                
extern void THROTTLE_runState(THROTTLE_Handle handle)
{
  THROTTLE_Obj *obj = (THROTTLE_Obj *)handle;


  if(obj->flagSw1)
  {
    obj->result = _IQ(0.0);
    obj->state = THROTTLE_CalMaxMin;
    obj->max_adc = obj->min_out;
    obj->min_adc = obj->max_out;
  }
  else
  {
    switch (obj->state)
    {
    case THROTTLE_CalMaxMin:

      if (obj->value > obj->max_adc)
        obj->max_adc = obj->value;
      else if (obj->value < obj->min_adc)
        obj->min_adc = obj->value;
      else if (obj->flagSw2)
        obj->state = THROTTLE_CalCalc;

    break;

    case THROTTLE_CalCalc:
    {
      obj->slope = _IQdiv((obj->max_out - obj->min_out),(obj->max_adc - obj->min_adc));
      obj->offset = obj->max_out - _IQmpy(obj->slope,obj->max_adc);
      
      obj->state = THROTTLE_Run;
    }
    break;

    case THROTTLE_Run:
    {
      _iq result = _IQmpy(obj->value,obj->slope) + obj->offset;
      obj->result = _IQsat(result,obj->max_out,obj->min_out);
    }
    break;
    }
  }


  return;
}


