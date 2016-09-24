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
#ifndef _OSC_H_
#define _OSC_H_

//! \file   drivers/osc/src/32b/f28x/f2802x/osc.h
//! \brief  Contains public interface to various functions related
//!         to the oscillator (OSC) object 
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

// drivers
#include "cpu.h"


// modules
#include "types.h"

//!
//!
//! \defgroup OSC OSC
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the base address of the oscillator (OSC) registers
//!
#define  OSC_BASE_ADDR                   (0x00007014)


//! \brief Defines the location of the COARSETRIM bits in the INTOSCnTRIM register
//!
#define  OSC_INTOSCnTRIM_COARSE_BITS     (255 << 0)

//! \brief Defines the location of the FINETRIM bits in the INTOSCnTRIM register
//!
#define  OSC_INTOSCnTRIM_FINE_BITS       (63 << 9)


// **************************************************************************
// the typedefs


//! \brief Enumeration to define the oscillator (OSC) source
//!
typedef enum
{
  OSC_Src_Internal=(0 << 0),  //!< Denotes an internal oscillator
  OSC_Src_External=(1 << 0)   //!< Denotes an external oscillator
} OSC_Src_e;


//! \brief Enumeration to define the oscillator (OSC) 2 source
//!
typedef enum
{
  OSC_Osc2Src_Internal=(0 << 1),  //!< Denotes an internal oscillator source for oscillator 2
  OSC_Osc2Src_External=(1 << 1)   //!< Denotes an external oscillator source for oscillator 2
} OSC_Osc2Src_e;


//! \brief Enumeration to define the oscillator (OSC) number
//!
typedef enum
{
  OSC_Number_1=1,  //!< Denotes oscillator number 1
  OSC_Number_2     //!< Denotes oscillator number 2
} OSC_Number_e;


//! \brief Defines the oscillator (OSC) object
//!
typedef struct _OSC_Obj_
{
    volatile uint16_t   INTOSC1TRIM;  //!< Internal Oscillator 1 Trim Register
    volatile uint16_t   rsvd_1;       //!< Reserved
    volatile uint16_t   INTOSC2TRIM;  //!< Internal Oscillator 2 Trim Register
} OSC_Obj;


//! \brief Defines the oscillator (OSC) handle
//!
typedef struct _OSC_Obj_ *OSC_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes



//! \brief     Initializes the oscillator (OSC) handle
//! \param[in] pMemory     A pointer to the base address of the FLASH registers
//! \param[in] numBytes    The number of bytes allocated for the FLASH object, bytes
//! \return    The flash (FLASH) object handle
extern OSC_Handle OSC_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the coarse trim value for a specified oscillator
//! \param[in] clkHandle  The oscillator (OSC) object handle
//! \param[in] oscNumber  The oscillator number
//! \param[in] trimValue  The coarse trim value
extern void OSC_setCoarseTrim(OSC_Handle clkHandle,
                              const OSC_Number_e oscNumber,
                              const uint_least8_t trimValue);


//! \brief     Sets the fine trim value for a specified oscillator
//! \param[in] clkHandle  The oscillator (OSC) object handle
//! \param[in] oscNumber  The oscillator number
//! \param[in] trimValue  The fine trim value
extern void OSC_setFineTrim(OSC_Handle clkHandle,
                            const OSC_Number_e oscNumber,
                            const uint_least8_t trimValue);


//! \brief     Sets the trim value for a specified oscillator
//! \param[in] clkHandle  The oscillator (OSC) object handle
//! \param[in] oscNumber  The oscillator number
//! \param[in] trimValue  The fine trim value
extern void OSC_setTrim(OSC_Handle clkHandle,
                        const OSC_Number_e oscNumber,
                        const uint16_t trimValue);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif  // end of _OSC_H_ definition

