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
//! \file   modules/queue/src/queue.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         queue manager routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "queue.h"


// needed for memset()
#include "string.h"


// **************************************************************************
// the globals

uint32_t   gEventIndex;

EVENT_Obj  gEvents[QUEUE_MAX_NUM_EVENTS];


// **************************************************************************
// the functions

QUEUE_Handle QUEUE_init(void *pMemory,const size_t numBytes)
{
  QUEUE_Handle handle;
  QUEUE_Obj *obj;
  

  if(numBytes < sizeof(QUEUE_Obj))
    return((QUEUE_Handle)NULL);


  // assign the handle
  handle = (QUEUE_Handle)pMemory;


  // assign the object
  obj = (QUEUE_Obj *)handle;


  // configure the queue
  obj->firstEvent = (EVENT_Handle)pMemory;
  obj->lastEvent = (EVENT_Handle)pMemory;


  // zero out the events
  gEventIndex = 0;
  (void)memset(&gEvents[0],0,sizeof(gEvents));

  return(handle);
} // end of QUEUE_init() function


void QUEUE_listen(QUEUE_Handle handle)
{

  for(;;)
    {

      // wait until the specific queue has an event
      while(QUEUE_isIdle(handle))
	{
          // allow some sleep time
          #ifndef MATLAB_SIMULATION
	  asm(" NOP ");   
	  asm(" NOP ");   
	  asm(" NOP ");   
	  asm(" NOP ");   
	  asm(" NOP ");   
          #endif
	}


      // execute event
      QUEUE_executeEvent(handle);
    }  // end of for() loop

} // end of QUEUE_listen() function


// end of file
