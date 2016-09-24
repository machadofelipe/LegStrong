/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
#ifndef _VIB_COMP_H_
#define _VIB_COMP_H_

//! \file   modules/vib_comp/src/32b/vib_comp.h
//! \brief  Contains the public interface to the 
//!         Vibration Compensation (VIB_COMP) module routines 
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

//!
//!
//! \defgroup VIB_COMP VIB_COMP
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup VIB_COMP_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif

// the includes

#include "IQmathLib.h"
#include "types.h"

// the typedefs

//! \brief Defines the VIB_COMP handle
//!
typedef struct _VIB_COMP_Obj_ *VIB_COMP_Handle;


// **************************************************************************
// the function prototypes


extern int16_t VIB_COMP_getAdvIndexDelta(VIB_COMP_Handle handle);

extern _iq VIB_COMP_getAlpha(VIB_COMP_Handle handle);

extern bool VIB_COMP_getFlag_enableOutput(VIB_COMP_Handle handle);

extern bool VIB_COMP_getFlag_enableUpdates(VIB_COMP_Handle handle);

extern int16_t VIB_COMP_getIndex(VIB_COMP_Handle handle);

//! \brief     Gets the size of the vibration compensation module in 16 bit words
//! \return    The size of the VIB_COMP object, in 16 bit words
extern size_t VIB_COMP_getSizeOfObject(void);

//! \brief     Initializes the vibration compensation module
//! \param[in] pMemory   A pointer to the vibration compensation object memory
//! \param[in] numBytes  The number of bytes allocated for the vibration compensation object, bytes
//! \return    The vibration compensation (VIB_COMP) object handle
extern VIB_COMP_Handle VIB_COMP_init(void *pMemory,const size_t numBytes);

//! \brief     Resets the vibration compensation module
//! \param[in] handle  The vibration compensation handle
extern void VIB_COMP_reset(VIB_COMP_Handle handle);

//! \brief     Runs the vibration compensation algorithm
//! \param[in] handle         The vibration compensation handle
//! \param[in] angle_mech_pu  The mechanical angle in per units from _IQ(0.0) to _IQ(1.0)
//! \param[in] Iq_in_pu       The measured Iq in per units
//! \return    The value to be used as a feed forward term in the speed controller
extern _iq VIB_COMP_run(VIB_COMP_Handle handle,const _iq angle_mech_pu,const _iq Iq_in_pu);

extern void VIB_COMP_setAdvIndexDelta(VIB_COMP_Handle handle,const int16_t adv_index_delta);

extern void VIB_COMP_setAlpha(VIB_COMP_Handle handle,const _iq alpha);

extern void VIB_COMP_setFlag_enableOutput(VIB_COMP_Handle handle,const bool state);

extern void VIB_COMP_setFlag_enableUpdates(VIB_COMP_Handle handle,const bool state);

extern void VIB_COMP_setIndex(VIB_COMP_Handle handle,const int16_t index);

extern void VIB_COMP_setParams(VIB_COMP_Handle handle,const _iq alpha,const int16_t adv_index_delta);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _VIB_COMP_H_ definition

