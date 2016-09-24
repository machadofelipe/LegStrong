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
#ifndef _ADC_OBJ_H_
#define _ADC_OBJ_H_

//! \file   drivers/adc/src/32b/f28x/adc_obj.h
//! \brief  Contains public interface to various functions related
//!         to the analog-to-digital converter (ADC) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// modules
#include "math.h"
#include "types.h"



//! \brief Defines the analog-to-digital converter (ADC) object
//!
typedef struct _ADC_Obj_
{
    volatile uint16_t      ADCRESULT[16];    //!< ADC result registers
    volatile uint16_t      resvd_1[26096];   //!< Reserved
    volatile uint16_t      ADCCTL1;          //!< ADC Control Register 1
    volatile uint16_t      rsvd_2[3];        //!< Reserved
    volatile uint16_t      ADCINTFLG;        //!< ADC Interrupt Flag Register
    volatile uint16_t      ADCINTFLGCLR;     //!< ADC Interrupt Flag Clear Register
    volatile uint16_t      ADCINTOVF;        //!< ADC Interrupt Overflow Register
    volatile uint16_t      ADCINTOVFCLR;     //!< ADC Interrupt Overflow Clear Register
    volatile uint16_t      INTSELxNy[5];     //!< ADC Interrupt Select x and y Register
    volatile uint16_t      rsvd_3[3];        //!< Reserved
    volatile uint16_t      SOCPRICTRL;       //!< ADC Start Of Conversion Priority Control Register
    volatile uint16_t      rsvd_4;           //!< Reserved
    volatile uint16_t      ADCSAMPLEMODE;    //!< ADC Sample Mode Register
    volatile uint16_t      rsvd_5;           //!< Reserved
    volatile uint16_t      ADCINTSOCSEL1;    //!< ADC Interrupt Trigger SOC Select 1 Register
    volatile uint16_t      ADCINTSOCSEL2;    //!< ADC Interrupt Trigger SOC Select 2 Register
    volatile uint16_t      rsvd_6[2];        //!< Reserved
    volatile uint16_t      ADCSOCFLG1;       //!< ADC SOC Flag 1 Register
    volatile uint16_t      rsvd_7;           //!< Reserved
    volatile uint16_t      ADCSOCFRC1;       //!< ADC SOC Force 1 Register
    volatile uint16_t      rsvd_8;           //!< Reserved
    volatile uint16_t      ADCSOCOVF1;       //!< ADC SOC Overflow 1 Register
    volatile uint16_t      rsvd_9;           //!< Reserved
    volatile uint16_t      ADCSOCOVFCLR1;    //!< ADC SOC Overflow Clear 1 Register
    volatile uint16_t      rsvd_10;          //!< Reserved
    volatile uint16_t      ADCSOCxCTL[16];   //!< ADC SOCx Control Registers
    volatile uint16_t      rsvd_11[16];      //!< Reserved
    volatile uint16_t      ADCREFTRIM;       //!< ADC Reference/Gain Trim Register
    volatile uint16_t      ADCOFFTRIM;       //!< ADC Offset Trim Register
    volatile uint16_t      resvd_12[13];     //!< Reserved
    volatile uint16_t      ADCREV;           //!< ADC Revision Register
} ADC_Obj;


//! \brief Defines the analog-to-digital converter (ADC) handle
//!
typedef struct _ADC_Obj_ *ADC_Handle;



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _ADC_OBJ_H_ definition
