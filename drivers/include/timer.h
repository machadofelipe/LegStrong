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
#ifndef _TIMER_H_
#define _TIMER_H_

//! \file   drivers/timer/src/32b/f28x/f2802x/timer.h
//!
//! \brief  Contains public interface to various functions related
//!         to the timer (TIMER) object 
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

#include "types.h"

//!
//!
//! \defgroup TIMER TIMER
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the base address of the timer (TIMER) 0 registers
//!
#define  TIMER0_BASE_ADDR                (0x00000C00)

//! \brief Defines the base address of the timer (TIMER) 1 registers
//!
#define  TIMER1_BASE_ADDR                (0x00000C08)

//! \brief Defines the base address of the timer (TIMER) 2 registers
//!
#define  TIMER2_BASE_ADDR                (0x00000C10)


//! \brief Defines the location of the TSS bits in the TCR register
//!
#define  TIMER_TCR_TSS_BITS               (1 << 4)

//! \brief Defines the location of the TRB bits in the TCR register
//!
#define  TIMER_TCR_TRB_BITS               (1 << 5)

//! \brief Defines the location of the FREESOFT bits in the TCR register
//!
#define  TIMER_TCR_FREESOFT_BITS          (3 << 10)

//! \brief Defines the location of the TIE bits in the TCR register
//!
#define  TIMER_TCR_TIE_BITS               (1 << 14)

//! \brief Defines the location of the TIF bits in the TCR register
//!
#define  TIMER_TCR_TIF_BITS               (1 << 15)


// **************************************************************************
// the typedefs


//! \brief Enumeration to define the timer (TIMER) emulation mode
//!
typedef enum
{
  TIMER_EmulationMode_StopAfterNextDecrement=(0 << 10),  //!< Denotes that the timer will stop after the next decrement
  TIMER_EmulationMode_StopAtZero=(1 << 10),              //!< Denotes that the timer will stop when it reaches zero
  TIMER_EmulationMode_RunFree=(2 << 10)                  //!< Denotes that the timer will run free
} TIMER_EmulationMode_e;


//! \brief Enumeration to define the timer (TIMER) status
//!
typedef enum
{
  TIMER_Status_CntIsNotZero=(0 << 15),  //!< Denotes that the counter is non-zero
  TIMER_Status_CntIsZero=(1 << 15)      //!< Denotes that the counter is zero
} TIMER_Status_e;


//! \brief Defines the timer (TIMER) object
//!
typedef struct _TIMER_Obj_
{
    volatile uint32_t  TIM;    //!< Timer Counter Register
    volatile uint32_t  PRD;    //!< Period Register
    volatile uint16_t  TCR;    //!< Timer Control Register
    volatile uint16_t  resvd_1;//!< Reserved
    volatile uint32_t  TPR;    //!< Timer Prescaler Register
} TIMER_Obj;


//! \brief Defines the timer (TIMER) handle
//!
typedef struct _TIMER_Obj_ *TIMER_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Clears the timer (TIMER) flag
//! \param[in] timerHandle   The timer (TIMER) object handle
extern void TIMER_clearFlag(TIMER_Handle timerHandle);


//! \brief     Disables the timer (TIMER) interrupt
//! \param[in] timerHandle   The timer (TIMER) object handle
extern void TIMER_disableInt(TIMER_Handle timerHandle);


//! \brief     Enables the timer (TIMER) interrupt
//! \param[in] timerHandle   The timer (TIMER) object handle
extern void TIMER_enableInt(TIMER_Handle timerHandle);


//! \brief     Gets the timer (TIMER) count
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \return    The timer (TIMER) count
static inline uint32_t TIMER_getCount(TIMER_Handle timerHandle)
{
  TIMER_Obj *volatile timer = (TIMER_Obj *)timerHandle;


  // get the count
  uint32_t cnt = timer->TIM;

  return(cnt);
} // end of TIMER_getCount() function


//! \brief     Gets the timer (TIMER) status
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \return    The timer (TIMER) status
extern TIMER_Status_e TIMER_getStatus(TIMER_Handle timerHandle);


//! \brief     Initializes the timer (TIMER) object handle
//! \param[in] pMemory     A pointer to the base address of the TIMER registers
//! \param[in] numBytes    The number of bytes allocated for the TIMER object, bytes
//! \return    The timer (CLK) object handle
extern TIMER_Handle TIMER_init(void *pMemory,const size_t numBytes);


//! \brief     Reloads the timer (TIMER) value 
//! \param[in] timerHandle   The timer (TIMER) object handle
static inline void TIMER_reload(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // clear the bits
    timer->TCR |= (uint16_t)TIMER_TCR_TRB_BITS;

    return;
} // end of TIMER_reload() function


//! \brief     Sets the timer (TIMER) decimation factor
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \param[in] decFactor     The timer decimation factor
extern void TIMER_setDecimationFactor(TIMER_Handle timerHandle,
                               const uint16_t decFactor);


//! \brief     Sets the timer (TIMER) emulation mode
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \param[in] mode          The emulation mode
extern void TIMER_setEmulationMode(TIMER_Handle timerHandle,
                            const TIMER_EmulationMode_e mode);


//! \brief     Gets the timer (TIMER) period
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \return    The timer (TIMER) period
static inline uint32_t TIMER_getPeriod(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // get the period
    uint32_t period = timer->PRD;

    return(period);
} // end of TIMER_getPeriod() function


//! \brief     Sets the timer (TIMER) period
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \param[in] period        The period
static inline void TIMER_setPeriod(TIMER_Handle timerHandle,
                                   const uint32_t period)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // set the bits
    timer->PRD = period;

    return;
} // end of TIMER_setPeriod() function


//! \brief     Sets the timer (TIMER) prescaler
//! \param[in] timerHandle   The timer (TIMER) object handle
//! \param[in] preScaler     The preScaler value
extern void TIMER_setPreScaler(TIMER_Handle timerHandle,
                        const uint16_t preScaler);


//! \brief     Starts the timer (TIMER)
//! \param[in] timerHandle   The timer (TIMER) object handle
static inline void TIMER_start(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // clear the bits
    timer->TCR &= (~(uint16_t)TIMER_TCR_TSS_BITS);

    return;
} // end of TIMER_start() function


//! \brief     Stops the timer (TIMER)
//! \param[in] timerHandle   The timer (TIMER) object handle
static inline void TIMER_stop(TIMER_Handle timerHandle)
{
    TIMER_Obj *timer = (TIMER_Obj *)timerHandle;


    // set the bits
    timer->TCR |= (uint16_t)TIMER_TCR_TSS_BITS;

    return;
} // end of TIMER_stop() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif  // end of _TIMER_H_ definition
