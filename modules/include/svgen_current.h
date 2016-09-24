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
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
#ifndef SVGENCURRENT_H_
#define SVGENCURRENT_H_
//! \file   modules/svgen/src/32b/svgen_current.h
//! \brief  Contains the public interface to the 
//!         Svgen Current module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "math.h"
#include "types.h"


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the typedefs

typedef enum
{
	use_all=0,		//!< Use all shunt measurements
	ignore_a,		//!< Ignore the A phase shunt measurement
	ignore_b,		//!< Ignore the B phase shunt measurement
	ignore_c,		//!< Ignore the C phase shunt measurement
	ignore_ab,		//!< Ignore the AB phase shunt measurement
	ignore_ac,		//!< Ignore the AC phase shunt measurement
	ignore_bc		//!< Ignore the BC phase shunt measurement
}	SVGENCURRENT_IgnoreShunt_e;


//! \brief Defines the Svgen Current object
//!
typedef struct _SVGENCURRENT_Obj_
{
  int16_t                       MinWidth;               //!< The maximum width where a valid measurement cannot be taken
  SVGENCURRENT_IgnoreShunt_e    IgnoreShunt;            //!< Output of what shunt or shunts to ignore
	
} SVGENCURRENT_Obj;


//! \brief Defines the Svgen Current handle
//!
typedef struct _SVGENCURRENT_Obj_ *SVGENCURRENT_Handle;


// **************************************************************************
// the function prototypes


//! \brief     Initializes the svgen current object
//! \param[in] *pMemory         Pointer in to the svgen current object
//! \param[in] numBytes         Size of the object
extern SVGENCURRENT_Handle SVGENCURRENT_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the minimum Duty Cycle width that the lower switch can be on before
//! \brief     the current data is invalid.
//! \param[in] svgencurrentHandle           The Svgen Current handle
//! \param[in] minwidth                     Integer value of the minimum number of pwm counts
static inline void SVGENCURRENT_setMinWidth(SVGENCURRENT_Handle svgencurrentHandle,const int16_t minwidth)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  svgencurrent->MinWidth = minwidth;

  return;
} // end of SVGENCURRENT_setMinWidth() function


//! \brief     Sets the ignore shunt value
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \param[in] ignoreShunt         The ignore shunt value
static inline void SVGENCURRENT_setIgnoreShunt(SVGENCURRENT_Handle svgencurrentHandle,const SVGENCURRENT_IgnoreShunt_e ignoreShunt)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  svgencurrent->IgnoreShunt = ignoreShunt;

  return;
} // end of SVGENCURRENT_setIgnoreShunt() function


//! \brief     Gets the ignore shunt value
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \return    Ignore shunt value
static inline SVGENCURRENT_IgnoreShunt_e SVGENCURRENT_getIgnoreShunt(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->IgnoreShunt);
} // end of SVGENCURRENT_getIgnoreShunt() function


//! \brief     Gets the minimum Duty Cycle width that the lower switch can be on before
//! \brief     the current data is invalid.
//! \param[in] svgencurrentHandle           The Svgen Current handle
//! \return    Integer value of the minimum number of pwm counts
static inline int16_t SVGENCURRENT_getMinWidth(SVGENCURRENT_Handle svgencurrentHandle)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  return(svgencurrent->MinWidth);
} // end of SVGENCURRENT_getMinWidth() function


//! \brief     Gets the svgen current module ignore shunt
//! \brief     In the pwm structure, the value variable is the on-time of the low fet.
//! \brief     A low value is a small on-time for the low switch of the bridge and thus a short current window.
//! \param[in] svgencurrentHandle  The Svgen Current handle
//! \param[in] cmp1                compare value 1
//! \param[in] cmp2                compare value 2
//! \param[in] cmp3                compare value 3
//! \param[in] cmpM1               active compare value 1, from mirror register
//! \param[in] cmpM2               active compare value 2, from mirror register
//! \param[in] cmpM3               active compare value 3, from mirror register
static inline void SVGENCURRENT_RunIgnoreShunt(SVGENCURRENT_Handle svgencurrentHandle, uint16_t cmp1, uint16_t cmp2, uint16_t cmp3, uint16_t cmpM1, uint16_t cmpM2, uint16_t cmpM3)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;
  uint16_t minWidth;

  minWidth = svgencurrent->MinWidth;

  uint16_t nextPulse1 = (cmp1 + cmpM1)>>1;
  uint16_t nextPulse2 = (cmp2 + cmpM2)>>1;
  uint16_t nextPulse3 = (cmp3 + cmpM3)>>1;

  if(nextPulse1 < minWidth)
    {
      if((nextPulse2 < minWidth) || ((cmp2 - cmp1) < minWidth))
        {
          svgencurrent->IgnoreShunt = ignore_ab;
        }
      else if((nextPulse3 < minWidth) || ((cmp3 - cmp1) < minWidth))
        {
          svgencurrent->IgnoreShunt = ignore_ac;
        }
      else
        {
          svgencurrent->IgnoreShunt = ignore_a;
        }
    }
  else if(nextPulse2 < minWidth)
    {
      if((nextPulse1 < minWidth) || ((cmp1 - cmp2) < minWidth))
        {
          svgencurrent->IgnoreShunt = ignore_ab;
        }
      else if((nextPulse3 < minWidth) || ((cmp3 - cmp2) < minWidth))
        {
          svgencurrent->IgnoreShunt = ignore_bc;
        }
      else
        {
          svgencurrent->IgnoreShunt = ignore_b;
        }
    }
  else if(nextPulse3 < minWidth)
    {
      if((nextPulse1 < minWidth) || ((cmp1 - cmp3) < minWidth))
        {
          svgencurrent->IgnoreShunt = ignore_ac;
        }
      else if((nextPulse2 < minWidth) || ((cmp2 - cmp3) < minWidth))
        {
          svgencurrent->IgnoreShunt = ignore_bc;
        }
      else
        {
          svgencurrent->IgnoreShunt = ignore_c;
        }
    }
  else
    {
      svgencurrent->IgnoreShunt = use_all;
    }

  return;
} // end of SVGENCURRENT_RunIgnoreShunt() function


//! \brief     Reconstructs the missed measured currents due to a small sampling window
//! \param[in] svgencurrentHandle         The svgen current handle
//! \param[in] pAdcData                   Pointer to the shunt currents
static inline void SVGENCURRENT_RunRegenCurrent(SVGENCURRENT_Handle svgencurrentHandle, MATH_vec3 *pAdcData)
{
  SVGENCURRENT_Obj *svgencurrent = (SVGENCURRENT_Obj *)svgencurrentHandle;

  _iq Ia = pAdcData->value[0];
  _iq Ib = pAdcData->value[1];
  _iq Ic = pAdcData->value[2];


  // select valid shunts and ignore one when needed
  if (svgencurrent->IgnoreShunt==ignore_a)
  {		// repair a based on b and c
    Ia = -Ib - Ic;       //Ia = -Ib - Ic;
  }
  else if (svgencurrent->IgnoreShunt==ignore_b)
  {		// repair b based on a and c
    Ib = -Ia - Ic;       //Ib = -Ia - Ic;
  }
  else if (svgencurrent->IgnoreShunt==ignore_c)
  {		// repair c based on a and b
    Ic = -Ia - Ib;       //Ic = -Ia - Ib;
  }
  else if (svgencurrent->IgnoreShunt==ignore_ab)
  {		// repair a and b based on c
    Ia = (-Ic)>>1;       //Ia = (-Ic)/2;
    Ib = Ia;              //Ib = Ia;
  }
  else if (svgencurrent->IgnoreShunt==ignore_ac)
  {		// repair a and c based on b
    Ia = (-Ib)>>1;       //Ia = (-Ib)/2;
    Ic = Ia;              //Ic = Ia;
  }
  else if (svgencurrent->IgnoreShunt==ignore_bc)
  {		// repair b and c based on a
    Ib = (-Ia)>>1;       //Ib = (-Ia)/2;
    Ic = Ib;              //Ic = Ib;
  }

  pAdcData->value[0] = Ia;
  pAdcData->value[1] = Ib;
  pAdcData->value[2] = Ic;

  return;
} // end of SVGENCURRENT_RunRegenCurrent() function

#ifdef __cplusplus
}
#endif // extern "C"

#endif /*SVGENCURRENT_H_*/
