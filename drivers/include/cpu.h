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
#ifndef __CPU_H__
#define __CPU_H__

//! \file   drivers/cpu/src/32b/f28x/f2802x/cpu.h
//! \brief  Contains public interface to various functions related
//!         to the central processing unit (CPU) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"

//!
//!
//! \defgroup CPU CPU
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Define to enable interrupts (legacy)
//!
#define  EINT   asm(" clrc INTM")

//! \brief Define to enable interrupts
//!
#define  ENABLE_INTERRUPTS    asm(" clrc INTM")

//! \brief Define to disable interrupts (legacy)
//!
#define  DINT   asm(" setc INTM")

//! \brief Define to disable interrupts
//!
#define  DISABLE_INTERRUPTS   asm(" setc INTM")

//! \brief Define to enable debug events
//!
#define  ERTM   asm(" clrc DBGM")

//! \brief Define to disable debug events
//!
#define  DRTM   asm(" setc DBGM")

//! \brief Define to allow protected register writes (legacy)
//!
#define  EALLOW asm(" EALLOW")

//! \brief Define to allow protected register writes
//!
#define  ENABLE_PROTECTED_REGISTER_WRITE_MODE  asm(" EALLOW")

//! \brief Define to disable protected register writes (legacy)
//!
#define  EDIS   asm(" EDIS")

//! \brief Define to disable protected register writes
//!
#define  DISABLE_PROTECTED_REGISTER_WRITE_MODE asm(" EDIS")

//! \brief Define for emulation stop 0
//!
#define  ESTOP0  asm(" ESTOP0")

//! \brief Define for entering IDLE mode
//!
#define  IDLE  asm(" IDLE")

//! \brief Defines the location of the INT1 bits in the IER register
//!
#define  CPU_IER_INT1_BITS          (1 << 0)

//! \brief Defines the location of the INT2 bits in the IER register
//!
#define  CPU_IER_INT2_BITS          (1 << 1)

//! \brief Defines the location of the INT3 bits in the IER register
//!
#define  CPU_IER_INT3_BITS          (1 << 2)

//! \brief Defines the location of the INT4 bits in the IER register
//!
#define  CPU_IER_INT4_BITS          (1 << 3)

//! \brief Defines the location of the INT5 bits in the IER register
//!
#define  CPU_IER_INT5_BITS          (1 << 4)

//! \brief Defines the location of the INT6 bits in the IER register
//!
#define  CPU_IER_INT6_BITS          (1 << 5)

//! \brief Defines the location of the INT7 bits in the IER register
//!
#define  CPU_IER_INT7_BITS          (1 << 6)

//! \brief Defines the location of the INT8 bits in the IER register
//!
#define  CPU_IER_INT8_BITS          (1 << 7)

//! \brief Defines the location of the INT9 bits in the IER register
//!
#define  CPU_IER_INT9_BITS          (1 << 8)

//! \brief Defines the location of the INT10 bits in the IER register
//!
#define  CPU_IER_INT10_BITS         (1 << 9)

//! \brief Defines the location of the INT11 bits in the IER register
//!
#define  CPU_IER_INT11_BITS         (1 << 10)

//! \brief Defines the location of the INT12 bits in the IER register
//!
#define  CPU_IER_INT12_BITS         (1 << 11)

//! \brief Defines the location of the INT13 bits in the IER register
//!
#define  CPU_IER_INT13_BITS         (1 << 12)

//! \brief Defines the location of the INT14 bits in the IER register
//!
#define  CPU_IER_INT14_BITS         (1 << 13)

//! \brief Defines the location of the DLOGINT bits in the IER register
//!
#define  CPU_IER_DLOGINT_BITS       (1 << 14)

//! \brief Defines the location of the RTOSINT bits in the IER register
//!
#define  CPU_IER_RTOSINT_BITS       (1 << 15)



//! \brief Defines the location of the INT1 bits in the IER register
//!
#define  CPU_IFR_INT1_BITS          (1 << 0)

//! \brief Defines the location of the INT2 bits in the IER register
//!
#define  CPU_IFR_INT2_BITS          (1 << 1)

//! \brief Defines the location of the INT3 bits in the IER register
//!
#define  CPU_IFR_INT3_BITS          (1 << 2)

//! \brief Defines the location of the INT4 bits in the IER register
//!
#define  CPU_IFR_INT4_BITS          (1 << 3)

//! \brief Defines the location of the INT5 bits in the IER register
//!
#define  CPU_IFR_INT5_BITS          (1 << 4)

//! \brief Defines the location of the INT6 bits in the IER register
//!
#define  CPU_IFR_INT6_BITS          (1 << 5)

//! \brief Defines the location of the INT7 bits in the IER register
//!
#define  CPU_IFR_INT7_BITS          (1 << 6)

//! \brief Defines the location of the INT8 bits in the IER register
//!
#define  CPU_IFR_INT8_BITS          (1 << 7)

//! \brief Defines the location of the INT9 bits in the IER register
//!
#define  CPU_IFR_INT9_BITS          (1 << 8)

//! \brief Defines the location of the INT10 bits in the IER register
//!
#define  CPU_IFR_INT10_BITS         (1 << 9)

//! \brief Defines the location of the INT11 bits in the IER register
//!
#define  CPU_IFR_INT11_BITS         (1 << 10)

//! \brief Defines the location of the INT12 bits in the IER register
//!
#define  CPU_IFR_INT12_BITS         (1 << 11)

//! \brief Defines the location of the INT13 bits in the IER register
//!
#define  CPU_IFR_INT13_BITS         (1 << 12)

//! \brief Defines the location of the INT14 bits in the IER register
//!
#define  CPU_IFR_INT14_BITS         (1 << 13)

//! \brief Defines the location of the DLOGINT bits in the IFR register
//!
#define  CPU_IFR_DLOGINT_BITS       (1 << 14)

//! \brief Defines the location of the RTOSINT bits in the IFR register
//!
#define  CPU_IFR_RTOSINT_BITS       (1 << 15)


//! \brief Defines the location of the INT1 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT1_BITS       (1 << 0)

//! \brief Defines the location of the INT2 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT2_BITS       (1 << 1)

//! \brief Defines the location of the INT3 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT3_BITS       (1 << 2)

//! \brief Defines the location of the INT4 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT4_BITS       (1 << 3)

//! \brief Defines the location of the INT5 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT5_BITS       (1 << 4)

//! \brief Defines the location of the INT6 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT6_BITS       (1 << 5)

//! \brief Defines the location of the INT7 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT7_BITS       (1 << 6)

//! \brief Defines the location of the INT8 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT8_BITS       (1 << 7)

//! \brief Defines the location of the INT9 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT9_BITS       (1 << 8)

//! \brief Defines the location of the INT10 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT10_BITS      (1 << 9)

//! \brief Defines the location of the INT11 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT11_BITS      (1 << 10)

//! \brief Defines the location of the INT12 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT12_BITS      (1 << 11)

//! \brief Defines the location of the INT13 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT13_BITS      (1 << 12)

//! \brief Defines the location of the INT14 bits in the DBGIER register
//!
#define  CPU_DBGIER_INT14_BITS      (1 << 13)

//! \brief Defines the location of the DLOGINT bits in the DBGIER register
//!
#define  CPU_DBGIER_DLOGINT_BITS    (1 << 14)

//! \brief Defines the location of the RTOSINT bits in the DBGIER register
//!
#define  CPU_DBGIER_RTOSINT_BITS    (1 << 15)



//! \brief Defines the location of the SXM bits in the ST0 register
//!
#define  CPU_ST0_SXM_BITS           (1 << 0)

//! \brief Defines the location of the OVM bits in the ST0 register
//!
#define  CPU_ST0_OVM_BITS           (1 << 1)

//! \brief Defines the location of the T bits in the ST0 register
//!
#define  CPU_ST0_TC_BITS            (1 << 2)

//! \brief Defines the location of the C bits in the ST0 register
//!
#define  CPU_ST0_C_BITS             (1 << 3)

//! \brief Defines the location of the Z bits in the ST0 register
//!
#define  CPU_ST0_Z_BITS             (1 << 4)

//! \brief Defines the location of the N bits in the ST0 register
//!
#define  CPU_ST0_N_BITS             (1 << 5)

//! \brief Defines the location of the V bits in the ST0 register
//!
#define  CPU_ST0_V_BITS             (1 << 6)

//! \brief Defines the location of the PW bits in the ST0 register
//!
#define  CPU_ST0_PW_BITS            (7 << 7)

//! \brief Defines the location of the OVCOVCU bits in the ST0 register
//!
#define  CPU_ST0_OVCOVCU_BITS       (63 << 10)



//! \brief Defines the location of the INTM bits in the ST1 register
//!
#define  CPU_ST1_INTM_BITS          (1 << 0)

//! \brief Defines the location of the DBGM bits in the ST1 register
//!
#define  CPU_ST1_DBGM_BITS          (1 << 1)

//! \brief Defines the location of the PAGE0 bits in the ST1 register
//!
#define  CPU_ST1_PAGE0_BITS         (1 << 2)

//! \brief Defines the location of the VMAP bits in the ST1 register
//!
#define  CPU_ST1_VMAP_BITS          (1 << 3)

//! \brief Defines the location of the SPA bits in the ST1 register
//!
#define  CPU_ST1_SPA_BITS           (1 << 4)

//! \brief Defines the location of the LOOP bits in the ST1 register
//!
#define  CPU_ST1_LOOP_BITS          (1 << 5)

//! \brief Defines the location of the EALLOW bits in the ST1 register
//!
#define  CPU_ST1_EALLOW_BITS        (1 << 6)

//! \brief Defines the location of the IDLESTAT bits in the ST1 register
//!
#define  CPU_ST1_IDLESTAT_BITS      (1 << 7)

//! \brief Defines the location of the AMODE bits in the ST1 register
//!
#define  CPU_ST1_AMODE_BITS         (1 << 8)

//! \brief Defines the location of the OBJMODE bits in the ST1 register
//!
#define  CPU_ST1_OBJMODE_BITS       (1 << 9)

//! \brief Defines the location of the MOM1MAP bits in the ST1 register
//!
#define  CPU_ST1_MOM1MAP_BITS       (1 << 11)

//! \brief Defines the location of the XF bits in the ST1 register
//!
#define  CPU_ST1_XF_BITS            (1 << 12)

//! \brief Defines the location of the ARP bits in the ST1 register
//!
#define  CPU_ST1_ARP_BITS           (7 << 13)


// **************************************************************************
// the typedefs

//! \brief Enumeration to define the external interrupt numbers
//!
typedef enum
{
  CPU_ExtIntNumber_1=0,      //!< Denotes external interrupt number 1
  CPU_ExtIntNumber_2,        //!< Denotes external interrupt number 2
  CPU_ExtIntNumber_3         //!< Denotes external interrupt number 3
} CPU_ExtIntNumber_e;


//! \brief Enumeration to define the interrupt numbers
//!
typedef enum
{
  CPU_IntNumber_1=(1 << 0),      //!< Denotes interrupt number 1
  CPU_IntNumber_2=(1 << 1),      //!< Denotes interrupt number 2
  CPU_IntNumber_3=(1 << 2),      //!< Denotes interrupt number 3
  CPU_IntNumber_4=(1 << 3),      //!< Denotes interrupt number 4
  CPU_IntNumber_5=(1 << 4),      //!< Denotes interrupt number 5
  CPU_IntNumber_6=(1 << 5),      //!< Denotes interrupt number 6
  CPU_IntNumber_7=(1 << 6),      //!< Denotes interrupt number 7
  CPU_IntNumber_8=(1 << 7),      //!< Denotes interrupt number 8
  CPU_IntNumber_9=(1 << 8),      //!< Denotes interrupt number 9
  CPU_IntNumber_10=(1 << 9),     //!< Denotes interrupt number 10
  CPU_IntNumber_11=(1 << 10),    //!< Denotes interrupt number 11
  CPU_IntNumber_12=(1 << 11),    //!< Denotes interrupt number 12
  CPU_IntNumber_13=(1 << 12),    //!< Denotes interrupt number 13
  CPU_IntNumber_14=(1 << 13)     //!< Denotes interrupt number 14
} CPU_IntNumber_e;


//! \brief Defines the central processing unit (CPU) object
//!
typedef struct _CPU_Obj_
{
  uint_least8_t   tmp;   //!< a filler value for the object
} CPU_Obj;


//! \brief Defines the central processing unit (CPU) handle
//!
typedef struct _CPU_Obj_ *CPU_Handle;


// **************************************************************************
// the globals


//! \brief Defines the CPU object
//!
extern CPU_Obj cpu;


//! \brief External reference to the interrupt flag register (IFR) register
//!
extern cregister volatile unsigned int IFR;


//! \brief External reference to the interrupt enable register (IER) register
//!
extern cregister volatile unsigned int IER;


// **************************************************************************
// the function prototypes

//! \brief     Clears all interrupt flags
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
extern void CPU_clearIntFlags(CPU_Handle cpuHandle);


//! \brief     Disables the debug interrupt
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
extern void CPU_disableDebugInt(CPU_Handle cpuHandle);


//! \brief     Disables global interrupts
//! \param[in] cpuHandle  The CPU handle
extern void CPU_disableGlobalInts(CPU_Handle cpuHandle);


//! \brief     Disables a specified interrupt number
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
//! \param[in] intNumber  The interrupt number
extern void CPU_disableInt(CPU_Handle cpuHandle,const CPU_IntNumber_e intNumber);


//! \brief     Disables all interrupts
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
extern void CPU_disableInts(CPU_Handle cpuHandle);


//! \brief     Disables protected register writes
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
extern void CPU_disableProtectedRegisterWrite(CPU_Handle cpuHandle);


//! \brief     Enables the debug interrupt
//! \param[in] cpuHandle  The CPU handle
extern void CPU_enableDebugInt(CPU_Handle cpuHandle);


//! \brief     Enables global interrupts
//! \param[in] cpuHandle  The CPU handle
extern void CPU_enableGlobalInts(CPU_Handle cpuHandle);


//! \brief     Enables a specified interrupt number
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
//! \param[in] intNumber  The interrupt number
extern void CPU_enableInt(CPU_Handle cpuHandle,const CPU_IntNumber_e intNumber);


//! \brief     Enables protected register writes
//! \param[in] cpuHandle  The central processing unit (CPU) object handle
extern void CPU_enableProtectedRegisterWrite(CPU_Handle cpuHandle);


//! \brief     Initializes the central processing unit (CPU) object handle
//! \param[in] pMemory    A pointer to the memory for the CPU object
//! \param[in] numBytes   The number of bytes allocated for the CPU object, bytes
//! \return    The central processing unit (CPU) object handle
extern CPU_Handle CPU_init(void *pMemory,const size_t numBytes);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif  // end of __CPU_H__ definition

