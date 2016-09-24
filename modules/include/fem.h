#ifndef _FEM_H_
#define _FEM_H_

//! \file   ~/sw/modules/fem/src/32b/fem.h
//! \brief  Contains the public interface to the 
//!         frequency of execution monitoring(FEM) module routines
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <math.h>

#include "types.h"


//!
//! \defgroup FEM

//!
//! \ingroup FEM
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines the frequency of execution monitoring (FEM) object
//!
typedef struct _FEM_Obj_
{
  uint32_t          timerPeriod_cnts;      //!< the timer period, cnts
  uint32_t          cnt_z0;                //!< the current timer count value, cnts
  uint32_t          cnt_z1;                //!< the previous timer count value, cnts
  uint32_t          deltaCnt;              //!< the latest delta count value, cnts

  uint32_t          maxDeltaCnt;           //!< the maximum delta count allowed, cnts
  uint32_t          minDeltaCnt;           //!< the minimum delta count allowed, cnts

  uint32_t          maxDeltaCntObserved;   //!< the maximum delta counts observed, cnts
  
  uint32_t          errorCnt;              //!< denotes the number of frequency errors that have been detected

  bool            flag_freqError;        //!< a flag to denote that a frequency error has occurred
} FEM_Obj;


//! \brief Defines the FEM handle
//!
typedef struct _FEM_Obj_ *FEM_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Gets the current count value
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The current timer count value
static inline uint32_t FEM_getCnt_z0(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->cnt_z0);
} // end of FEM_getCnt_z0() function


//! \brief     Gets the previous count value
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The previous ount value
static inline uint32_t FEM_getCnt_z1(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->cnt_z1);
} // end of FEM_getCnt_z1() function


//! \brief     Gets the latest delta count measured, cnts
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The latest delta counts measured, cnts
static inline uint32_t FEM_getDeltaCnt(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->deltaCnt);
} // end of FEM_getDeltaCnt() function


//! \brief     Gets the frequency error count
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The number of frequency errors
static inline uint32_t FEM_getErrorCnt(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->errorCnt);
} // end of FEM_getErrorCnt() function


//! \brief     Gets the state of the frequency error flag
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The desired state
static inline bool FEM_getFlag_freqError(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->flag_freqError);
} // end of FEM_getFlag_freqError() function


//! \brief     Gets the maximum delta count allowed, cnts
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The maximum delta count allowed, cnts
static inline uint32_t FEM_getMaxDeltaCnt(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->maxDeltaCnt);
} // end of FEM_getMaxDeltaCnt() function


//! \brief     Gets the maximum delta count observed, cnts
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The maximum delta count allowed, cnts
static inline uint32_t FEM_getMaxDeltaCntObserved(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->maxDeltaCntObserved);
} // end of FEM_getMaxDeltaCntObserved() function


//! \brief     Gets the minimum delta count allowed, cnts
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The minimum delta count allowed, cnts
static inline uint32_t FEM_getMinDeltaCnt(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->minDeltaCnt);
} // end of FEM_getMinDeltaCnts() function


//! \brief     Gets the timer period, cnts
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The timer period, cnts
static inline uint32_t FEM_getTimerPeriod(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->timerPeriod_cnts);
} // end of FEM_getTimerPeriod() function


//! \brief     Initializes the frequency of execution monitoring (FEM) object
//! \param[in] pMemory    A pointer to the base address of the object
//! \param[in] numBytes   The object size, bytes
//! \return    The handle to the object
extern FEM_Handle FEM_init(void *pMemory,const size_t numBytes);


//! \brief     Gets the state of the frequency error flag
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \return    The desired state
static inline bool FEM_isFreqError(FEM_Handle handle)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  return(obj->flag_freqError);
} // end of FEM_isFreqError() function


//! \brief     Sets the current count value
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \param[in] cnt     The current count value
static inline void FEM_setCnt_z0(FEM_Handle handle,const uint32_t cnt)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->cnt_z0 = cnt;

  return;
} // end of FEM_setCnt_z0() function


//! \brief     Sets the previous count value
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \param[in] cnt     The previous count value
static inline void FEM_setCnt_z1(FEM_Handle handle,const uint32_t cnt)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->cnt_z1 = cnt;

  return;
} // end of FEM_setCnt_z1() function


//! \brief     Sets the delta count value
//! \param[in] handle    The frequency of execution monitoring (FEM) handle
//! \param[in] deltaCnt  The delta count value
static inline void FEM_setDeltaCnt(FEM_Handle handle,const uint32_t deltaCnt)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->deltaCnt = deltaCnt;

  return;
} // end of FEM_setDeltaCnt() function


//! \brief     Sets the error count
//! \param[in] handle     The frequency of execution monitoring (FEM) handle
//! \param[in] numErrors  The number of frequency errors
static inline void FEM_setErrorCnt(FEM_Handle handle,const uint32_t numErrors)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->errorCnt = numErrors;

  return;
} // end of FEM_setErrorCnt() function


//! \brief     Sets the state of the frequency error flag
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
//! \param[in] state   The desired state
static inline void FEM_setFlag_freqError(FEM_Handle handle,const bool state)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->flag_freqError = state;

  return;
} // end of FEM_setFlag_freqError() function


//! \brief     Sets the maximum delta count, cnts
//! \param[in] handle       The frequency of execution monitoring (FEM) handle
//! \param[in] maxDeltaCnt  The maximum delta count, cnts
static inline void FEM_setMaxDeltaCnt(FEM_Handle handle,const uint32_t maxDeltaCnt)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->maxDeltaCnt = maxDeltaCnt;

  return;
} // end of FEM_setMaxDeltaCnt() function


//! \brief     Sets the maximum delta count observed, cnts
//! \param[in] handle       The frequency of execution monitoring (FEM) handle
//! \param[in] maxDeltaCnt  The maximum delta count observed, cnts
static inline void FEM_setMaxDeltaCntObserved(FEM_Handle handle,const uint32_t maxDeltaCnt)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->maxDeltaCntObserved = maxDeltaCnt;

  return;
} // end of FEM_setMaxDeltaCntObserved() function


//! \brief     Sets the minimum delta count, cnts
//! \param[in] handle       The frequency of execution monitoring (FEM) handle
//! \param[in] minDeltaCnt  The minimum delta count, cnts
static inline void FEM_setMinDeltaCnt(FEM_Handle handle,const uint32_t minDeltaCnt)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->minDeltaCnt = minDeltaCnt;

  return;
} // end of FEM_setMinDeltaCnt() function


//! \brief     Sets the frequency of execution monitoring parameters, cnts
//! \param[in] handle             The frequency of execution monitoring (FEM) handle
//! \param[in] timerFreq_Hz       The timer frequency, Hz
//! \param[in] timerPeriod_cnts   The timer period, cnts
//! \param[in] spFreq_Hz          The setpoint frequency, Hz
//! \param[in] maxError_Hz        The maximum frequency error, Hz
void FEM_setParams(FEM_Handle handle,
                     const float_t timerFreq_Hz,
                     const uint32_t timerPeriod_cnts,
                     const float_t spFreq_Hz,
                     const float_t maxError_Hz);


//! \brief     Sets the timer period, cnts
//! \param[in] handle            The frequency of execution monitoring (FEM) handle
//! \param[in] timerPeriod_cnts  The timer period, cnts
static inline void FEM_setTimerPeriod(FEM_Handle handle,const uint32_t timerPeriod_cnts)
{
  FEM_Obj *obj = (FEM_Obj *)handle;

  obj->timerPeriod_cnts = timerPeriod_cnts;

  return;
} // end of FEM_setTimerPeriod() function


//! \brief     Increments the error counter
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
static inline void FEM_incrErrorCnt(FEM_Handle handle)
{
  uint32_t errorCnt = FEM_getErrorCnt(handle);

  errorCnt++;

  FEM_setErrorCnt(handle,errorCnt);

  return;
} // end of FEM_incrErrorCnt() function


//! \brief     Runs the frequency of execution monitoring
//! \param[in] handle  The frequency of execution monitoring (FEM) handle
static inline void FEM_run(FEM_Handle handle)
{
  uint32_t timerPeriod = FEM_getTimerPeriod(handle);
  uint32_t maxDeltaCnt = FEM_getMaxDeltaCnt(handle);
  uint32_t maxDeltaCntObserved = FEM_getMaxDeltaCntObserved(handle);
  uint32_t minDeltaCnt = FEM_getMinDeltaCnt(handle);
  uint32_t cnt_z0 = FEM_getCnt_z0(handle);
  uint32_t cnt_z1 = FEM_getCnt_z1(handle);

  uint32_t deltaCnt;


  // compute the actual frequency
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


  // store the value 
  FEM_setDeltaCnt(handle,deltaCnt);


  // keep track of the maximum delta count observed
  if(deltaCnt > maxDeltaCntObserved)
    {
      // store the value 
      FEM_setMaxDeltaCntObserved(handle,deltaCnt);
    }


  // check for an error
  if((deltaCnt < minDeltaCnt) || (deltaCnt > maxDeltaCnt))
    {
      // set the flag
      FEM_setFlag_freqError(handle,true);

      // increment the counter
      FEM_incrErrorCnt(handle);
    }
  else
    {
      // set the flag
      FEM_setFlag_freqError(handle,false);
    }

  return;
} // end of FEM_run() function


//! \brief     Updates the current and previous count values
//! \param[in] handle    The frequency of execution monitoring (FEM) handle
//! \param[in] timerCnt  The current count value
static inline void FEM_updateCnts(FEM_Handle handle,const uint32_t cnt)
{
  uint32_t cnt_z0 = FEM_getCnt_z0(handle);

  FEM_setCnt_z1(handle,cnt_z0);
  FEM_setCnt_z0(handle,cnt);

  return;
} // end of FEM_updateCnts() function



#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _FEM_H_ definition


