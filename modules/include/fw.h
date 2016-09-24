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
#ifndef _FW_H_
#define _FW_H_

//! \file   modules/fw/src/32b/fw.h
//! \brief  Contains public interface to various functions related
//!         to the FW object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "IQmathLib.h"
#include "types.h"

//!
//!
//! \defgroup FW FW
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the Field Weakening (FW) increment delta correction
//!
#define FW_INC_DELTA                     (1000)


//! \brief Defines the Field Weakening (FW) decrement delta correction
//!
#define FW_DEC_DELTA                     (1000)


//! \brief Defines the number of isr ticks per field weakening clock tick
//!
#define FW_NUM_ISR_TICKS_PER_CTRL_TICK   (10)


// **************************************************************************
// the typedefs
  

//! \brief Defines the field weakening (FW) data
//!
typedef struct _FW_Obj_
{
  uint32_t     numIsrTicksPerFwTick;   //!< Defines the number of isr clock ticks per field weakening clock tick

  uint32_t     counter_fw;             //!< the field weakening counter
  
  _iq          delta_inc;              //!< the field weakening delta increment of Id reference
  _iq          delta_dec;              //!< the field weakening delta decrement of Id reference

  _iq          refValue;               //!< the reference input value
  _iq          fbackValue;             //!< the feedback input value

  _iq          output;                 //!< the output of field weakening
	
  _iq          outMin;                 //!< the minimum output value allowed for the FW controller
  _iq          outMax;                 //!< the maximum output value allowed for the FW controller
  
  bool       flag_enableFw;          //!< a flag to enable field weakening
} FW_Obj;


//! \brief Defines the FW handle
//!
typedef struct _FW_Obj_ *FW_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Initializes the field weakening (FW) object
//! \param[in] pMemory   A pointer to the memory for the field weakening object
//! \param[in] numBytes  The number of bytes allocated for the field weakening object, bytes
//! \return    The field weakening (FW) object handle
extern FW_Handle FW_init(void *pMemory, const size_t numBytes);


//! \brief      Gets the minimum and maximum output value allowed in the FW controller
//! \param[in]  fwHandle  The FW controller handle
//! \param[out] pOutMin   The pointer to the minimum output value allowed
//! \param[out] pOutMax   The pointer to the maximum output value allowed
static inline void FW_getMinMax(FW_Handle fwHandle,_iq *pOutMin,_iq *pOutMax)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  *pOutMin = fw->outMin;
  *pOutMax = fw->outMax;

  return;
} // end of FW_getMinMax() function


//! \brief     Sets the minimum and maximum output value allowed in the FW controller
//! \param[in] fwHandle  The FW controller handle
//! \param[in] outMin    The minimum output value allowed
//! \param[in] outMax    The maximum output value allowed
static inline void FW_setMinMax(FW_Handle fwHandle,const _iq outMin,const _iq outMax)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->outMin = outMin;
  fw->outMax = outMax;

  return;
} // end of FW_setMinMax() function


//! \brief     Configures the deltas of the field weakening (FW) object
//! \param[in] fwHandle   The Field Weakening handle
//! \param[in] delta_inc  The delta increment to Id reference
//! \param[in] delta_dec  The delta decrement to Id reference
static inline void FW_setDeltas(FW_Handle fwHandle, const _iq delta_inc, const _iq delta_dec)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->delta_inc = delta_inc;
  
  fw->delta_dec = delta_dec;
 
  return;
} // end of FW_setDeltas() function


//! \brief     Configures the number of ISR ticks per field weakening tick of the field weakening (FW) object
//! \param[in] fwHandle              The Field Weakening handle
//! \param[in] numIsrTicksPerFwTick  The number of ISR ticks per field weakening ticks
static inline void FW_setNumIsrTicksPerFwTick(FW_Handle fwHandle, const uint32_t numIsrTicksPerFwTick)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->numIsrTicksPerFwTick = numIsrTicksPerFwTick;
 
  return;
} // end of FW_setNumIsrTicksPerFwTick() function


//! \brief     Gets the number of ISR ticks per field weakening tick of the field weakening (FW) object
//! \param[in] fwHandle   The Field Weakening handle
//! \return    The number of ISR ticks per field weakening ticks
static inline uint32_t FW_getNumIsrTicksPerFwTick(FW_Handle fwHandle)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  return(fw->numIsrTicksPerFwTick);
} // end of FW_getNumIsrTicksPerFwTick() function


//! \brief     Clears the counter of the field weakening (FW) object
//! \param[in] fwHandle  The Field Weakening handle
static inline void FW_clearCounter(FW_Handle fwHandle)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->counter_fw = 0;
 
  return;
} // end of FW_clearCounter() function


//! \brief     Increments the counter of the field weakening (FW) object
//! \param[in] fwHandle  The Field Weakening handle
static inline void FW_incCounter(FW_Handle fwHandle)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->counter_fw++;
 
  return;
} // end of FW_incCounter() function


//! \brief     Returns the counter of the field weakening (FW) object
//! \param[in] fwHandle  The Field Weakening handle
static inline uint32_t FW_getCounter(FW_Handle fwHandle)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  return(fw->counter_fw);
} // end of FW_getCounter() function


//! \brief     Sets the enable flag of the field weakening (FW) object
//! \param[in] fwHandle  The Field Weakening handle
//! \param[in] state     The Field Weakening enable state
static inline void FW_setFlag_enableFw(FW_Handle fwHandle, const bool state)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->flag_enableFw = state;
  
  return;
} // end of FW_setFlag_enableFw() function


//! \brief     Gets the enable flag of the field weakening (FW) object
//! \param[in] fwHandle  The Field Weakening handle
//! \return    The Field Weakening enable state
static inline bool FW_getFlag_enableFw(FW_Handle fwHandle)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  return(fw->flag_enableFw);
} // end of FW_getFlag_enableFw() function


//! \brief     Sets the start value in the FW controller
//! \param[in] fwHandle  The FW controller handle
//! \param[in] output    The start value for the FW controller
static inline void FW_setOutput(FW_Handle fwHandle,const _iq output)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  fw->output = output;

  return;
} // end of FW_setOutput() function


//! \brief     Gets the output value in the FW controller
//! \param[in] fwHandle  The FW controller handle
//! \return    The output value for the FW controller
static inline _iq FW_getOutput(FW_Handle fwHandle)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  return(fw->output);
} // end of FW_getOutput() function


//! \brief     Runs the FW controller
//! \param[in] fwHandle    The FW controller handle
//! \param[in] refValue    The reference value to the controller
//! \param[in] fbackValue  The feedback value to the controller
//! \param[in] pOutValue   The pointer to the controller output value
static inline void FW_run(FW_Handle fwHandle,const _iq refValue,const _iq fbackValue,_iq *pOutValue)
{
  FW_Obj *fw = (FW_Obj *)fwHandle;

  _iq Error;
  _iq output = fw->output;
  _iq delta_inc = fw->delta_inc;
  _iq delta_dec = fw->delta_inc;


  Error = refValue - fbackValue;

  if(Error < 0)
    {
	  output = output - delta_dec;                // Calculate the output for negative error
	}
  else if(Error > 0)
    {
	  output = output + delta_inc;                // Calculate the output for positive error
	}
  
  output = _IQsat(output,fw->outMax,fw->outMin);  // Saturate the output

  fw->output = output;                            // store the output
  fw->refValue = refValue;
  fw->fbackValue = fbackValue;

  *pOutValue = output;

  return;
} // end of FW_run() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _FW_H_ definition

