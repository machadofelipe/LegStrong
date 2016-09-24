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
#ifndef _HALLBLDC_H_
#define _HALLBLDC_H_

//! \file   modules/hallbldc/src/32b/hallbldc.h
//! \brief  Contains the public interface to the 
//!         Clarke transform (CLARKE) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "IQmathLib.h"
#include "math.h"
#include "types.h"

//!
//!
//! \defgroup HALLBLDC HALLBLDC
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

typedef enum
{
  BLDC_CW = 0,          //!< ClockWise
  BLDC_CCW  = 1         //!< CounterClockWise
} BLDC_DIR_e;


//! \brief Defines the CLARKE object
//!
typedef struct _HALLBLDC_Obj_
{
  uint_least8_t  prevState_bin;     //!< the previous hall state for determining direction
  _iq            angle_pu;          //!< the pu angle of the current hall state

  uint_least8_t  sensorType;         //!< 120 deg or 60 deg
  uint_least8_t  angleSelect;

} HALLBLDC_Obj;


//! \brief Defines the CLARKE handle
//!
typedef struct _HALLBLDC_Obj_ *HALLBLDC_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Gets the angle of the hall feedback
//! \param[in] handle  The Hall Bldc handle
//! \return    The hall rotor angle
static inline _iq HALLBLDC_getAngle_pu(HALLBLDC_Handle handle)
{
  HALLBLDC_Obj *obj = (HALLBLDC_Obj *)handle;

  return(obj->angle_pu);
} // end of HALLBLDC_getAngle_pu() function


//! \brief     Initializes the Hall BLDC transform module
//! \param[in] pMemory   A pointer to the memory for the Clarke object
//! \param[in] numBytes  The number of bytes allocated for the Clarke object, bytes
//! \return The Hall Bldc (HALLBLDC) object handle
extern HALLBLDC_Handle HALLBLDC_init(void *pMemory,const size_t numBytes);


//! \brief     Runs the hall bldc module for hall inputs
//! \param[in] handle  The hall bldc handle
//! \param[in] hallState     The hall state 1 thru 6, A phase hall is MSB, C phase hall is LSB
//! \param[in] pCurrentVec        The pointer to the current vector
static inline void HALLBLDC_run(HALLBLDC_Handle handle,const uint_least8_t hallState)  //,MATH_vec3 *pCurrentVec)
{
  HALLBLDC_Obj *obj = (HALLBLDC_Obj *)handle;


  switch(hallState)
  {
    case 5:
    {
      if(obj->angleSelect == 0)
      {
        obj->angle_pu = _IQ(180.0/360.0);
      }
      else if(obj->angleSelect == 1)
      {
        obj->angle_pu = _IQ(240.0/360.0);
      }

      break;
    }
    case 4:
    {
      if(obj->angleSelect == 0)
      {
        obj->angle_pu = _IQ(240.0/360.0);
      }
      else if(obj->angleSelect == 1)
      {
        obj->angle_pu = _IQ(300.0/360.0);
      }

      break;
    }
    case 6:
    {
      if(obj->angleSelect == 0)
      {
        obj->angle_pu = _IQ(300.0/360.0);
      }
      else if(obj->angleSelect == 1)
      {
        obj->angle_pu = _IQ(0.0/360.0);
      }

      break;
    }
    case 2:
    {
      if(obj->angleSelect == 0)
      {
        obj->angle_pu = _IQ(0.0/360.0);
      }
      else if(obj->angleSelect == 1)
      {
        obj->angle_pu = _IQ(60.0/360.0);
      }

      break;
    }
    case 3:
    {
      if(obj->angleSelect == 0)
      {
        obj->angle_pu = _IQ(60.0/360.0);
      }
      else if(obj->angleSelect == 1)
      {
        obj->angle_pu = _IQ(120.0/360.0);
      }

      break;
    }
    case 1:
    {
      if(obj->angleSelect == 0)
      {
        obj->angle_pu = _IQ(120.0/360.0);
      }
      else if(obj->angleSelect == 1)
      {
        obj->angle_pu = _IQ(180.0/360.0);
      }

      break;
    }
  }
  obj->prevState_bin = hallState;

  
  return;
} // end of HALLBLDC_run() function



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _HALLBLDC_H_ definition

