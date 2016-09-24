//! \file   ~/sw/modules/cpu_usage/src/32b/cpu_usage.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         CPU usage (CPU_USAGE) module routines
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "cpu_usage.h"


// **************************************************************************
// the globals


// **************************************************************************
// the functions

CPU_USAGE_Handle CPU_USAGE_init(void *pMemory,const size_t numBytes)
{
  CPU_USAGE_Handle handle;

  if(numBytes < sizeof(CPU_USAGE_Obj))
    return((CPU_USAGE_Handle)NULL);

  // assign the handle
  handle = (CPU_USAGE_Handle)pMemory;

  return(handle);
} // end of CPU_USAGE_init() function

void CPU_USAGE_setParams(CPU_USAGE_Handle handle,
                         const uint32_t timerPeriod_cnts,
                         const uint32_t numDeltaCntsAvg)
{
  CPU_USAGE_setTimerPeriod(handle,timerPeriod_cnts);

  CPU_USAGE_setCnt_z0(handle,0);
  CPU_USAGE_setCnt_z1(handle,0);
  CPU_USAGE_setDeltaCnt(handle,0);

  CPU_USAGE_setDeltaCntAcc(handle,0);
  CPU_USAGE_setDeltaCntAccNum(handle,0);
  CPU_USAGE_setDeltaCntAccNumMax(handle,numDeltaCntsAvg);

  CPU_USAGE_setMinDeltaCntObserved(handle,timerPeriod_cnts);
  CPU_USAGE_setAvgDeltaCntObserved(handle,0);
  CPU_USAGE_setMaxDeltaCntObserved(handle,0);

  CPU_USAGE_setFlag_resetStats(handle,false);

  return;
} // end of CPU_USAGE_setParams() function

// end of file
