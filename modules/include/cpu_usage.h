#ifndef _CPU_USAGE_H_
#define _CPU_USAGE_H_

//! \file   ~/sw/modules/cpu_usage/src/32b/cpu_usage.h
//! \brief  Contains the public interface to the 
//!         CPU usage (CPU_USAGE) module routines
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <math.h>

#include "types.h"


//!
//! \defgroup CPU_USAGE

//!
//! \ingroup CPU_USAGE
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines the CPU usage (CPU_USAGE) object
//!
typedef struct _CPU_USAGE_Obj_
{
  uint32_t          timerPeriod_cnts;      //!< the timer period, cnts
  uint32_t          cnt_z0;                //!< the current timer count value, cnts
  uint32_t          cnt_z1;                //!< the previous timer count value, cnts
  uint32_t          deltaCnt;              //!< the latest delta count value, cnts

  uint32_t          deltaCntAcc;           //!< the accumulated delta count values, cnts
  uint32_t          deltaCntAccNum;           //!< the number of accumulated delta count values, num
  uint32_t          deltaCntAccNumMax;        //!< the maximum number of accumulated delta count values, num

  uint32_t          minDeltaCntObserved;   //!< the minimum delta counts observed, cnts
  uint32_t          avgDeltaCntObserved;   //!< the average delta counts observed, cnts
  uint32_t          maxDeltaCntObserved;   //!< the maximum delta counts observed, cnts
  
  bool              flag_resetStats;       //!< a flag to reset all measured data
} CPU_USAGE_Obj;


//! \brief Defines the CPU_USAGE handle
//!
typedef struct _CPU_USAGE_Obj_ *CPU_USAGE_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Gets the timer period, cnts
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The timer period, cnts
static inline uint32_t CPU_USAGE_getTimerPeriod(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->timerPeriod_cnts);
} // end of CPU_USAGE_getTimerPeriod() function


//! \brief     Gets the current count value
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The current timer count value
static inline uint32_t CPU_USAGE_getCnt_z0(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->cnt_z0);
} // end of CPU_USAGE_getCnt_z0() function


//! \brief     Gets the previous count value
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The previous ount value
static inline uint32_t CPU_USAGE_getCnt_z1(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->cnt_z1);
} // end of CPU_USAGE_getCnt_z1() function


//! \brief     Gets the latest delta count measured, cnts
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The latest delta counts measured, cnts
static inline uint32_t CPU_USAGE_getDeltaCnt(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->deltaCnt);
} // end of CPU_USAGE_getDeltaCnt() function


//! \brief     Gets the accumulated delta counts, cnts
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The accumulated delta counts, cnts
static inline uint32_t CPU_USAGE_getDeltaCntAcc(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->deltaCntAcc);
} // end of CPU_USAGE_getDeltaCntAcc() function


//! \brief     Gets the number of accumulated delta counts, num
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The number of accumulated delta counts, num
static inline uint32_t CPU_USAGE_getDeltaCntAccNum(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->deltaCntAccNum);
} // end of CPU_USAGE_getDeltaCntAccNum() function


//! \brief     Gets the maximum number of accumulated delta counts, num
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The maximum number of accumulated delta counts, num
static inline uint32_t CPU_USAGE_getDeltaCntAccNumMax(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->deltaCntAccNumMax);
} // end of CPU_USAGE_getDeltaCntAccNumMax() function


//! \brief     Gets the minimum delta count observed, cnts
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The minimum delta count allowed, cnts
static inline uint32_t CPU_USAGE_getMinDeltaCntObserved(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->minDeltaCntObserved);
} // end of CPU_USAGE_getMinDeltaCntObserved() function


//! \brief     Gets the average delta count observed, cnts
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The average delta count allowed, cnts
static inline uint32_t CPU_USAGE_getAvgDeltaCntObserved(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->avgDeltaCntObserved);
} // end of CPU_USAGE_getAvgDeltaCntObserved() function


//! \brief     Gets the maximum delta count observed, cnts
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The maximum delta count allowed, cnts
static inline uint32_t CPU_USAGE_getMaxDeltaCntObserved(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->maxDeltaCntObserved);
} // end of CPU_USAGE_getMaxDeltaCntObserved() function


//! \brief     Gets the state of the reset stats flag
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \return    The flag state
static inline bool CPU_USAGE_getFlag_resetStats(CPU_USAGE_Handle handle)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  return(obj->flag_resetStats);
} // end of CPU_USAGE_getFlag_resetStats() function


//! \brief     Initializes the CPU usage (CPU_USAGE) object
//! \param[in] pMemory    A pointer to the base address of the object
//! \param[in] numBytes   The object size, bytes
//! \return    The handle to the object
extern CPU_USAGE_Handle CPU_USAGE_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the timer period, cnts
//! \param[in] handle            The CPU usage (CPU_USAGE) handle
//! \param[in] timerPeriod_cnts  The timer period, cnts
static inline void CPU_USAGE_setTimerPeriod(CPU_USAGE_Handle handle,const uint32_t timerPeriod_cnts)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->timerPeriod_cnts = timerPeriod_cnts;

  return;
} // end of CPU_USAGE_setTimerPeriod() function


//! \brief     Sets the current count value
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \param[in] cnt     The current count value
static inline void CPU_USAGE_setCnt_z0(CPU_USAGE_Handle handle,const uint32_t cnt)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->cnt_z0 = cnt;

  return;
} // end of CPU_USAGE_setCnt_z0() function


//! \brief     Sets the previous count value
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \param[in] cnt     The previous count value
static inline void CPU_USAGE_setCnt_z1(CPU_USAGE_Handle handle,const uint32_t cnt)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->cnt_z1 = cnt;

  return;
} // end of CPU_USAGE_setCnt_z1() function


//! \brief     Sets the delta count value
//! \param[in] handle    The CPU usage (CPU_USAGE) handle
//! \param[in] deltaCnt  The delta count value
static inline void CPU_USAGE_setDeltaCnt(CPU_USAGE_Handle handle,const uint32_t deltaCnt)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->deltaCnt = deltaCnt;

  return;
} // end of CPU_USAGE_setDeltaCnt() function


//! \brief     Sets the accumulated delta counts value
//! \param[in] handle       The CPU usage (CPU_USAGE) handle
//! \param[in] deltaCntAcc  The accumulated delta counts value
static inline void CPU_USAGE_setDeltaCntAcc(CPU_USAGE_Handle handle,const uint32_t deltaCntAcc)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->deltaCntAcc = deltaCntAcc;

  return;
} // end of CPU_USAGE_setDeltaCntAcc() function


//! \brief     Sets the number of accumulated delta counts
//! \param[in] handle          The CPU usage (CPU_USAGE) handle
//! \param[in] deltaCntAccNum  The number of accumulated delta counts
static inline void CPU_USAGE_setDeltaCntAccNum(CPU_USAGE_Handle handle,const uint32_t deltaCntAccNum)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->deltaCntAccNum = deltaCntAccNum;

  return;
} // end of CPU_USAGE_setDeltaCntAccNum() function


//! \brief     Sets the maximum number of accumulated delta counts
//! \param[in] handle             The CPU usage (CPU_USAGE) handle
//! \param[in] deltaCntAccNumMax  The maximum number of accumulated delta counts
static inline void CPU_USAGE_setDeltaCntAccNumMax(CPU_USAGE_Handle handle,const uint32_t deltaCntAccNumMax)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->deltaCntAccNumMax = deltaCntAccNumMax;

  return;
} // end of CPU_USAGE_setDeltaCntAccNumMax() function


//! \brief     Sets the minimum delta count observed, cnts
//! \param[in] handle       The CPU usage (CPU_USAGE) handle
//! \param[in] minDeltaCnt  The minimum delta count observed, cnts
static inline void CPU_USAGE_setMinDeltaCntObserved(CPU_USAGE_Handle handle,const uint32_t minDeltaCnt)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->minDeltaCntObserved = minDeltaCnt;

  return;
} // end of CPU_USAGE_setMinDeltaCntObserved() function


//! \brief     Sets the average delta count observed, cnts
//! \param[in] handle       The CPU usage (CPU_USAGE) handle
//! \param[in] avgDeltaCnt  The average delta count observed, cnts
static inline void CPU_USAGE_setAvgDeltaCntObserved(CPU_USAGE_Handle handle,const uint32_t avgDeltaCnt)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->avgDeltaCntObserved = avgDeltaCnt;

  return;
} // end of CPU_USAGE_setAvgDeltaCntObserved() function


//! \brief     Sets the maximum delta count observed, cnts
//! \param[in] handle       The CPU usage (CPU_USAGE) handle
//! \param[in] maxDeltaCnt  The maximum delta count observed, cnts
static inline void CPU_USAGE_setMaxDeltaCntObserved(CPU_USAGE_Handle handle,const uint32_t maxDeltaCnt)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->maxDeltaCntObserved = maxDeltaCnt;

  return;
} // end of CPU_USAGE_setMaxDeltaCntObserved() function


//! \brief     Sets the state of the reset stats flag
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
//! \param[in] state   The desired state
static inline void CPU_USAGE_setFlag_resetStats(CPU_USAGE_Handle handle,const bool state)
{
  CPU_USAGE_Obj *obj = (CPU_USAGE_Obj *)handle;

  obj->flag_resetStats = state;

  return;
} // end of CPU_USAGE_setFlag_resetStats() function


//! \brief     Sets the CPU usage module parameters
//! \param[in] handle             The CPU usage (CPU_USAGE) handle
//! \param[in] timerPeriod_cnts   The timer period, cnts
//! \param[in] numDeltaCntsAvg    The number of delta accumulations for the average calculation
void CPU_USAGE_setParams(CPU_USAGE_Handle handle,
                         const uint32_t timerPeriod_cnts,
                         const uint32_t numDeltaCntsAvg);


//! \brief     Runs the CPU usage module
//! \param[in] handle  The CPU usage (CPU_USAGE) handle
static inline void CPU_USAGE_run(CPU_USAGE_Handle handle)
{
  uint32_t timerPeriod = CPU_USAGE_getTimerPeriod(handle);
  uint32_t minDeltaCntObserved = CPU_USAGE_getMinDeltaCntObserved(handle);
  uint32_t avgDeltaCntObserved = CPU_USAGE_getAvgDeltaCntObserved(handle);
  uint32_t maxDeltaCntObserved = CPU_USAGE_getMaxDeltaCntObserved(handle);
  uint32_t deltaCntAcc = CPU_USAGE_getDeltaCntAcc(handle);
  uint32_t deltaCntAccNum = CPU_USAGE_getDeltaCntAccNum(handle);
  uint32_t deltaCntAccNumMax = CPU_USAGE_getDeltaCntAccNumMax(handle);
  uint32_t cnt_z0 = CPU_USAGE_getCnt_z0(handle);
  uint32_t cnt_z1 = CPU_USAGE_getCnt_z1(handle);

  bool flag_resetStats = CPU_USAGE_getFlag_resetStats(handle);

  uint32_t deltaCnt;


  // compute the actual delta counts
  // handle wrap around of the timer count
  // NOTE: count down timer
  if(cnt_z0 > cnt_z1)
    {
      deltaCnt = cnt_z1 + timerPeriod - cnt_z0 + 1;
    }
  else
    {
      deltaCnt = cnt_z1 - cnt_z0 + 1;
    }

  // accumulate delta counts for the average calculation
  deltaCntAcc += deltaCnt;

  // increment the number of accumulations
  deltaCntAccNum++;
  
  // calculate average counts if number of accumulations matches the max number
  if(deltaCntAccNum >= deltaCntAccNumMax)
    {
      // compute the average
      avgDeltaCntObserved = deltaCntAcc/deltaCntAccNum;

      // reset accumulation and number of accumulations
      deltaCntAcc = 0;
      deltaCntAccNum = 0;
	}


  // keep track of the minimum delta count observed
  if(deltaCnt < minDeltaCntObserved)
    {
      minDeltaCntObserved = deltaCnt;
    }

  // keep track of the maximum delta count observed
  if(deltaCnt > maxDeltaCntObserved)
    {
      maxDeltaCntObserved = deltaCnt;
    }

  // reset statistics if reset flag is true
  if(flag_resetStats == true)
    {
      cnt_z0 = 0;
      cnt_z1 = 0;
      deltaCnt = 0;

      deltaCntAcc = 0;
      deltaCntAccNum = 0;

      minDeltaCntObserved = timerPeriod;
      avgDeltaCntObserved = 0;
      maxDeltaCntObserved = 0;

      flag_resetStats = false;
    }
	  

  // store the values
  CPU_USAGE_setDeltaCnt(handle,deltaCnt);
  CPU_USAGE_setDeltaCntAcc(handle,deltaCntAcc);
  CPU_USAGE_setDeltaCntAccNum(handle,deltaCntAccNum);
  CPU_USAGE_setMinDeltaCntObserved(handle,minDeltaCntObserved);
  CPU_USAGE_setAvgDeltaCntObserved(handle,avgDeltaCntObserved);
  CPU_USAGE_setMaxDeltaCntObserved(handle,maxDeltaCntObserved);
  CPU_USAGE_setFlag_resetStats(handle, flag_resetStats);


  return;
} // end of FEM_run() function


//! \brief     Updates the current and previous count values
//! \param[in] handle    The CPU usage (CPU_USAGE) handle
//! \param[in] timerCnt  The current count value
static inline void CPU_USAGE_updateCnts(CPU_USAGE_Handle handle,const uint32_t cnt)
{
  uint32_t cnt_z0 = CPU_USAGE_getCnt_z0(handle);

  CPU_USAGE_setCnt_z1(handle,cnt_z0);
  CPU_USAGE_setCnt_z0(handle,cnt);

  return;
} // end of CPU_USAGE_updateCnts() function


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CPU_USAGE_H_ definition


