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
//! \file   drivers/clk/src/32b/f28x/f2802x/clk.c
//! \brief  Contains the various functions related to the 
//!         clock object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "clk.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


void CLK_disableAdcClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR0 &= (~CLK_PCLKCR0_ADCENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableAdcClock() function


void CLK_disableClkIn(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_XCLKINOFF_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableClkIn() function


void CLK_disableCompClock(CLK_Handle clkHandle,const CLK_CompNumber_e compNumber)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  // clear the bits
  clk->PCLKCR3 &= (~compNumber);

  return;
} // end of CLK_disableCompClock() function


void CLK_disableCpuTimerClock(CLK_Handle clkHandle,const CLK_CpuTimerNumber_e cpuTimerNumber)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR3 &= (~cpuTimerNumber);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableCpuTimerClock() function


void CLK_disableCrystalOsc(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_XTALOSCOFF_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableCrystalOsc() function


void CLK_disableEcap1Clock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR1 &= (~CLK_PCLKCR1_ECAP1ENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableEcap1Clock() function


void CLK_disableGpioInputClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR3 &= (~CLK_PCLKCR3_GPIOINENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableGpioInputClock() function


void CLK_disableHrPwmClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR0 &= (~CLK_PCLKCR0_HRPWMENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableHrPwmClock() function


void CLK_disableI2cClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR0 &= (~CLK_PCLKCR0_I2CAENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableI2cClock() function


void CLK_disableOsc1(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_INTOSC1OFF_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableOsc1() function


void CLK_disableOsc1HaltMode(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_INTOSC1HALTI_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableOsc1HaltMode() function


void CLK_disableOsc2(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_INTOSC2OFF_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableOsc2() function


void CLK_disableOsc2HaltMode(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_INTOSC2HALTI_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableOsc2HaltMode() function


void CLK_disablePwmClock(CLK_Handle clkHandle,const PWM_Number_e pwmNumber)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR1 &= (~(1 << pwmNumber));

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disablePwmClock() function


void CLK_disableSciaClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR0 &= (~CLK_PCLKCR0_SCIAENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableSciaClock() function


void CLK_disableSpiaClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR0 &= (~CLK_PCLKCR0_SPIAENCLK_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableSpiaClock() function


void CLK_disableTbClockSync(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->PCLKCR0 &= (~CLK_PCLKCR0_TBCLKSYNC_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableTbClockSync() function


void CLK_disableWatchDogHaltMode(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->CLKCTL |= CLK_CLKCTL_WDHALTI_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_disableWatchDogHaltMode() function


void CLK_enableAdcClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR0 |= CLK_PCLKCR0_ADCENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableAdcClock() function


void CLK_enableCompClock(CLK_Handle clkHandle,const CLK_CompNumber_e compNumber)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR3 |= compNumber;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableCompClock() function


void CLK_enableCpuTimerClock(CLK_Handle clkHandle,const CLK_CpuTimerNumber_e cpuTimerNumber)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR3 |= cpuTimerNumber;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableCpuTimerClock() function


void CLK_enableCrystalOsc(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_XTALOSCOFF_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableCrystalOsc() function


void CLK_enableEcap1Clock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR1 |= CLK_PCLKCR1_ECAP1ENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableEcap1Clock() function


void CLK_enableClkIn(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_XCLKINOFF_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableClkIn() function


void CLK_enableGpioInputClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR3 |= CLK_PCLKCR3_GPIOINENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableGpioInputClock() function


void CLK_enableHrPwmClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR0 |= CLK_PCLKCR0_HRPWMENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableHrPwmClock() function


void CLK_enableI2cClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR0 |= CLK_PCLKCR0_I2CAENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableI2cClock() function


void CLK_enableOsc1(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_INTOSC1OFF_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableOsc1() function


void CLK_enableOsc1HaltMode(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_INTOSC1HALTI_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableOsc1HaltMode() function


void CLK_enableOsc2(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_INTOSC2OFF_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableOsc2() function


void CLK_enableOsc2HaltMode(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_INTOSC2HALTI_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableOsc2HaltMode() function


void CLK_enablePwmClock(CLK_Handle clkHandle,const PWM_Number_e pwmNumber)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR1 |= 1 << pwmNumber;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enablePwmClock() function


void CLK_enableSciaClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR0 |= CLK_PCLKCR0_SCIAENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableSciaClock() function


void CLK_enableSpiaClock(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR0 |= CLK_PCLKCR0_SPIAENCLK_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableSpiaClock() function


void CLK_enableTbClockSync(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  clk->PCLKCR0 |= CLK_PCLKCR0_TBCLKSYNC_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableTbClockSync() function


void CLK_enableWatchDogHaltMode(CLK_Handle clkHandle)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_WDHALTI_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_enableWatchDogHaltMode() function


CLK_Handle CLK_init(void *pMemory,const size_t numBytes)
{
  CLK_Handle clkHandle;


  if(numBytes < sizeof(CLK_Obj))
    return((CLK_Handle)NULL);

  // assign the handle
  clkHandle = (CLK_Handle)pMemory;

  return(clkHandle);
} // end of CLK_init() function


void CLK_setClkOutPreScaler(CLK_Handle clkHandle,
                            const CLK_ClkOutPreScaler_e preScaler)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->XCLK &= (~CLK_XCLK_XCLKOUTDIV_BITS);

  // set the bits
  clk->XCLK |= preScaler;
  
  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setClkOutPreScaler() function


void CLK_setLowSpdPreScaler(CLK_Handle clkHandle,
                            const CLK_LowSpdPreScaler_e preScaler)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
    clk->LOSPCP = preScaler;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setLowSpdPreScaler() function


void CLK_setOscSrc(CLK_Handle clkHandle,const CLK_OscSrc_e src)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;
  

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_OSCCLKSRCSEL_BITS);

  // set the bits
  clk->CLKCTL |= src;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setOscSrc() function
                      

void CLK_setOsc2Src(CLK_Handle clkHandle,const CLK_Osc2Src_e src)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;
  

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_OSCCLKSRC2SEL_BITS);

  // set the bits
  clk->CLKCTL |= src;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setOsc2Src() function


void CLK_setXClkInSrc(CLK_Handle clkHandle, const CLK_XClkInSrc_e src)
{
	CLK_Obj *clk = (CLK_Obj *)clkHandle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	// clear the bits
	clk->XCLK &= (~CLK_XCLK_XCLKINSEL_BITS);

	//set the bits
	clk->XCLK |= src;

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

	return;
} //end of CLK_setXCLKINSrc() function
                      

void CLK_setTimer2PreScale(CLK_Handle clkHandle,
                           const CLK_Timer2PreScaler_e preScaler)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;
  

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_TMR2CLKPRESCALE_BITS);

  // set the bits
  clk->CLKCTL |= preScaler;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setTimer2PreScaler() function
                      

void CLK_setTimer2Src(CLK_Handle clkHandle,const CLK_Timer2Src_e src)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;
  

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_TMR2CLKSRCSEL_BITS);

  // set the bits
  clk->CLKCTL |= src;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setTimer2Src() function
                      

void CLK_setWatchDogSrc(CLK_Handle clkHandle,const CLK_WdClkSrc_e src)
{
  CLK_Obj *clk = (CLK_Obj *)clkHandle;
  

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  clk->CLKCTL &= (~CLK_CLKCTL_WDCLKSRCSEL_BITS);

  // set the bits
  clk->CLKCTL |= src;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of CLK_setWatchDogSrc() function
                      


// end of file
