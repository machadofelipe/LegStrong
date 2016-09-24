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
#ifndef _PLL_H_
#define _PLL_H_

//! \file   drivers/pll/src/32b/f28x/f2802x/pll.h
//!
//! \brief  Contains public interface to various functions related
//!         to the phase-locked loop (PLL) object 
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"

#include "cpu.h"


//!
//!
//! \defgroup PLL PLL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the base address of the phase lock loop (PLL) registers
//!
#define  PLL_BASE_ADDR                   (0x00007011)


//! \brief Defines the location of the DIV bits in the PLLCR register
//!
#define  PLL_PLLCR_DIV_BITS              (15 << 0)


//! \brief Defines the location of the PLLLOCKS bits in the PLLSTS register
//!
#define PLL_PLLSTS_PLLLOCKS_BITS         (1 << 0)

//! \brief Defines the location of the PLLOFF bits in the PLLSTS register
//!
#define PLL_PLLSTS_PLLOFF_BITS           (1 << 2)

//! \brief Defines the location of the MCLKSTS bits in the PLLSTS register
//!
#define PLL_PLLSTS_MCLKSTS_BITS          (1 << 3)

//! \brief Defines the location of the MCLKCLR bits in the PLLSTS register
//!
#define PLL_PLLSTS_MCLKCLR_BITS          (1 << 4)

//! \brief Defines the location of the OSCOFF bits in the PLLSTS register
//!
#define PLL_PLLSTS_OSCOFF_BITS           (1 << 5)

//! \brief Defines the location of the MCLKOFF bits in the PLLSTS register
//!
#define PLL_PLLSTS_MCLKOFF_BITS          (1 << 6)

//! \brief Defines the location of the DIVSEL bits in the PLLSTS register
//!
#define PLL_PLLSTS_DIVSEL_BITS           (3 << 7)

//! \brief Defines the location of the NORMRDYE bits in the PLLSTS register
//!
#define PLL_PLLSTS_NORMRDYE_BITS         (1 << 15)


// **************************************************************************
// the typedefs


//! \brief Enumeration to define the phase lock loop (PLL) clock frequency
//!
typedef enum
{
  PLL_ClkFreq_5_MHz=(1 << 0),       //!< Denotes a clock frequency of 5 MHz
  PLL_ClkFreq_10_MHz=(2 << 0),      //!< Denotes a clock frequency of 10 MHz
  PLL_ClkFreq_15_MHz=(3 << 0),      //!< Denotes a clock frequency of 15 MHz
  PLL_ClkFreq_20_MHz=(4 << 0),      //!< Denotes a clock frequency of 20 MHz
  PLL_ClkFreq_25_MHz=(5 << 0),      //!< Denotes a clock frequency of 25 MHz
  PLL_ClkFreq_30_MHz=(6 << 0),      //!< Denotes a clock frequency of 30 MHz
  PLL_ClkFreq_35_MHz=(7 << 0),      //!< Denotes a clock frequency of 35 MHz
  PLL_ClkFreq_40_MHz=(8 << 0),      //!< Denotes a clock frequency of 40 MHz
  PLL_ClkFreq_45_MHz=(9 << 0),      //!< Denotes a clock frequency of 45 MHz
  PLL_ClkFreq_50_MHz=(10 << 0),     //!< Denotes a clock frequency of 50 MHz
  PLL_ClkFreq_55_MHz=(11 << 0),     //!< Denotes a clock frequency of 55 MHz
  PLL_ClkFreq_60_MHz=(12 << 0)      //!< Denotes a clock frequency of 60 MHz
} PLL_ClkFreq_e;


//! \brief Enumeration to define the phase lock loop (PLL) divide select
//!
typedef enum
{
    PLL_DivideSelect_ClkIn_by_4=(0 << 7),  //!< Denotes a divide select of CLKIN/4
    PLL_DivideSelect_ClkIn_by_2=(2 << 7),   //!< Denotes a divide select of CLKIN/2
    PLL_DivideSelect_ClkIn_by_1=(3 << 7)    //!< Denotes a divide select of CLKIN/1
} PLL_DivideSelect_e;


//! \brief Enumeration to define the phase lock loop (PLL) clock status
//!
typedef enum
{
    PLL_ClkStatus_Normal=(0 << 3),     //!< Denotes a normal clock
    PLL_ClkStatus_Missing=(1 << 3)     //!< Denotes a missing clock
} PLL_ClkStatus_e;


//! \brief Enumeration to define the phase lock loop (PLL) clock lock status
//!
typedef enum
{
    PLL_LockStatus_Locking=(0 << 0),   //!< Denotes that the system is locking to the clock
    PLL_LockStatus_Done=(1 << 0)       //!< Denotes that the system is locked to the clock
} PLL_LockStatus_e;


//! \brief Defines the phase lock loop (PLL) object
//!
typedef struct _PLL_Obj_
{
    volatile uint16_t   PLLSTS;       //!< PLL Status Register
    volatile uint16_t   rsvd_1;       //!< Reserved
    volatile uint16_t   PLLLOCKPRD;   //!< PLL Lock Period Register
    volatile uint16_t   rsvd_2[13];   //!< Reserved
    volatile uint16_t   PLLCR;        //!< PLL Control Register
} PLL_Obj;


//! \brief Defines the phase lock loop (PLL) handle
//!
typedef struct _PLL_Obj_ *PLL_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Disables the phase lock loop (PLL)
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_disable(PLL_Handle pllHandle);


//! \brief     Disables the clock detect logic
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_disableClkDetect(PLL_Handle pllHandle);


//! \brief     Disables the NORMRDY signal
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_disableNormRdy(PLL_Handle pllHandle);


//! \brief     Disables the oscillator
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_disableOsc(PLL_Handle pllHandle);


//! \brief     Enables the phase lock loop (PLL)
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_enable(PLL_Handle pllHandle);


//! \brief     Enables the clock detect logic
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_enableClkDetect(PLL_Handle pllHandle);


//! \brief     Enables the NORMRDY signal
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_enableNormRdy(PLL_Handle pllHandle);


//! \brief     Enables the oscillator
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_enableOsc(PLL_Handle pllHandle);


//! \brief     Gets the phase lock loop (PLL) clock frequency
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \return    The clock frequency
extern PLL_ClkFreq_e PLL_getClkFreq(PLL_Handle pllHandle);


//! \brief     Gets the phase lock loop (PLL) clock status
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \return    The clock status
extern PLL_ClkStatus_e PLL_getClkStatus(PLL_Handle pllHandle);


//! \brief     Gets the phase lock loop (PLL) divide select value
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \return    The divide select value
extern PLL_DivideSelect_e PLL_getDivideSelect(PLL_Handle pllHandle);


//! \brief     Gets the phase lock loop (PLL) lock status
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \return    The lock status
extern PLL_LockStatus_e PLL_getLockStatus(PLL_Handle pllHandle);


//! \brief     Initializes the phase lock loop (PLL) object handle
//! \param[in] pMemory     A pointer to the base address of the PLL registers
//! \param[in] numBytes    The number of bytes allocated for the PLL object, bytes
//! \return    The phase lock loop (PLL) object handle
extern PLL_Handle PLL_init(void *pMemory,const size_t numBytes);


//! \brief     Resets the phase lock loop (PLL) clock detect logic
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
extern void PLL_resetClkDetect(PLL_Handle pllHandle);


//! \brief     Sets the phase lock loop (PLL) clock frequency
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \param[in] freq       The clock frequency
extern void PLL_setClkFreq(PLL_Handle pllHandle,const PLL_ClkFreq_e freq);


//! \brief     Sets the phase lock loop (PLL) divide select value
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \param[in] divSelect  The divide select value
extern void PLL_setDivideSelect(PLL_Handle pllHandle,const PLL_DivideSelect_e divSelect);


//! \brief     Sets the phase lock loop (PLL) lock time
//! \param[in] pllHandle  The phase lock loop (PLL) object handle
//! \param[in] lockPeriod The lock period, cycles
extern void PLL_setLockPeriod(PLL_Handle pllHandle,const uint16_t lockPeriod);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif  // end of _PLL_H_ definition

