//! \file   ~/sw/modules/fem/src/32b/fem.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         frequency of execution monitoring (FEM) module routines
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "fem.h"


// **************************************************************************
// the globals


// **************************************************************************
// the functions

FEM_Handle FEM_init(void *pMemory,const size_t numBytes)
{
  FEM_Handle handle;

  if(numBytes < sizeof(FEM_Obj))
    return((FEM_Handle)NULL);

  // assign the handle
  handle = (FEM_Handle)pMemory;

  return(handle);
} // end of FEM_init() function


void FEM_setParams(FEM_Handle handle,
                     const float_t timerFreq_Hz,
                     const uint32_t timerPeriod_cnts,
                     const float_t spFreq_Hz,
                     const float_t maxFreqError_Hz)
{
  uint32_t maxDeltaCnt = (uint32_t)(timerFreq_Hz/(spFreq_Hz - maxFreqError_Hz));
  uint32_t minDeltaCnt = (uint32_t)(timerFreq_Hz/(spFreq_Hz + maxFreqError_Hz));

  FEM_setTimerPeriod(handle,timerPeriod_cnts);

  FEM_setCnt_z0(handle,0);
  FEM_setCnt_z1(handle,0);
  FEM_setDeltaCnt(handle,0);

  FEM_setMaxDeltaCnt(handle,maxDeltaCnt);
  FEM_setMaxDeltaCntObserved(handle,0);
  FEM_setMinDeltaCnt(handle,minDeltaCnt);

  FEM_setErrorCnt(handle,0);
  FEM_setFlag_freqError(handle,false);

  return;
} // end of FEM_setParams() function

// end of file
