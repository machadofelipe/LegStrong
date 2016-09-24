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
//! \file   drivers/adc/src/32b/f28x/f2802x/adc.c
//! \brief  Contains the various functions related to the
//!         analog-to-digital converter (ADC) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include "adc.h"


// assembly file
extern void usDelay(uint32_t Count);


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void ADC_disable(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  adc->ADCCTL1 &= (~ADC_ADCCTL1_ADCENABLE_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_disable() function


void ADC_disableBandGap(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_ADCBGPWD_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_disableBandGap() function


void ADC_disableInt(ADC_Handle adcHandle,const ADC_IntNumber_e intNumber)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;
  uint_least8_t regNumber = intNumber >> 1;
  uint16_t clearValue = ADC_INTSELxNy_INTE_BITS << 
    (ADC_INTSELxNy_NUMBITS_PER_REG - (((intNumber+1) & 0x1) << ADC_INTSELxNy_LOG2_NUMBITS_PER_REG));


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->INTSELxNy[regNumber] &= (~clearValue);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_disableInt() function


void ADC_disableRefBuffers(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_ADCREFPWD_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_disableRefBuffers() function


void ADC_enable(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 |= ADC_ADCCTL1_ADCENABLE_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enable() function


void ADC_enableBandGap(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 |= ADC_ADCCTL1_ADCBGPWD_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enableBandGap() function


void ADC_enableInt(ADC_Handle adcHandle,const ADC_IntNumber_e intNumber)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;
  uint_least8_t regNumber = intNumber >> 1;
  uint_least8_t lShift = ADC_INTSELxNy_NUMBITS_PER_REG - (((intNumber+1) & 0x1) << ADC_INTSELxNy_LOG2_NUMBITS_PER_REG);
  uint16_t setValue = ADC_INTSELxNy_INTE_BITS << lShift;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the value
  adc->INTSELxNy[regNumber] |= setValue;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enableInt() function


void ADC_enableRefBuffers(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 |= ADC_ADCCTL1_ADCREFPWD_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enableRefBuffers() function


// current sampled last
ADC_Handle ADC_init(void *pMemory,const size_t numBytes)
{
  ADC_Handle adcHandle;


  if(numBytes < sizeof(ADC_Obj))
    return((ADC_Handle)NULL);


  // assign the handle
  adcHandle = (ADC_Handle)pMemory;

  return(adcHandle);
} // end of ADC_init() function


void ADC_powerDown(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_ADCPWDN_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_powerDown() function


void ADC_powerUp(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 |= ADC_ADCCTL1_ADCPWDN_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_powerUp() function


void ADC_reset(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 |= (uint16_t)ADC_ADCCTL1_RESET_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_reset() function


void ADC_setSampleOverlapMode(ADC_Handle adcHandle, ADC_ADCCTL2_ADCNONOVERLAP_e OverLap)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  if(OverLap == ADC_ADCCTL2_Overlap)
    adc->ADCCTL2 &= ~((uint16_t)(ADC_ADCCTL2_ADCNONOVERLAP_BITS));
  else
    adc->ADCCTL2 |= (uint16_t)ADC_ADCCTL2_ADCNONOVERLAP_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSampleOverlapMode() function


void ADC_setIntMode(ADC_Handle adcHandle,const ADC_IntNumber_e intNumber,const ADC_IntMode_e intMode)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;
  uint_least8_t regNumber = intNumber >> 1;
  uint_least8_t lShift = (ADC_INTSELxNy_NUMBITS_PER_REG - (((intNumber+1) & 0x1) << ADC_INTSELxNy_LOG2_NUMBITS_PER_REG));
  uint16_t clearValue = ADC_INTSELxNy_INTCONT_BITS << lShift;
  uint16_t setValue = intMode << lShift;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->INTSELxNy[regNumber] &= ~(clearValue);


  // set the bits
  adc->INTSELxNy[regNumber] |= setValue;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setIntMode() function


void ADC_setIntPulseGenMode(ADC_Handle adcHandle,const ADC_IntPulseGenMode_e pulseMode)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_INTPULSEPOS_BITS);


  // set the bits
  adc->ADCCTL1 |= pulseMode;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setIntPulseGenMode() function


void ADC_setIntSrc(ADC_Handle adcHandle,const ADC_IntNumber_e intNumber,const ADC_IntSrc_e intSrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;
  uint_least8_t regNumber = intNumber >> 1;
  uint_least8_t lShift = (ADC_INTSELxNy_NUMBITS_PER_REG - (((intNumber+1) & 0x1) << ADC_INTSELxNy_LOG2_NUMBITS_PER_REG));
  uint16_t clearValue = ADC_INTSELxNy_INTSEL_BITS << lShift;
  uint16_t setValue = intSrc << lShift;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;


  // clear the bits
  adc->INTSELxNy[regNumber] &= ~(clearValue);


  // set the bits
  adc->INTSELxNy[regNumber] |= setValue;


  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setIntSrc() function


void ADC_setSampleMode(ADC_Handle adcHandle,const ADC_SampleMode_e sampleMode)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(sampleMode & ADC_ADCSAMPLEMODE_SEPARATE_FLAG) // separate
    {
      adc->ADCSAMPLEMODE &= (~(sampleMode - ADC_ADCSAMPLEMODE_SEPARATE_FLAG));
    }
  else
    {
      adc->ADCSAMPLEMODE |= sampleMode;
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSampleMode() function


void ADC_setSocChanNumber(ADC_Handle adcHandle,const ADC_SocNumber_e socNumber,const ADC_SocChanNumber_e chanNumber)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCSOCxCTL[socNumber] &= (~ADC_ADCSOCxCTL_CHSEL_BITS);


  // set the bits
  adc->ADCSOCxCTL[socNumber] |= chanNumber;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSocChanNumber() function


void ADC_setSocSampleDelay(ADC_Handle adcHandle,const ADC_SocNumber_e socNumber,const ADC_SocSampleDelay_e sampleDelay)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCSOCxCTL[socNumber] &= (~ADC_ADCSOCxCTL_ACQPS_BITS);


  // set the bits
  adc->ADCSOCxCTL[socNumber] |= sampleDelay;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSocSampleDelay() function


void ADC_setSocTrigSrc(ADC_Handle adcHandle,const ADC_SocNumber_e socNumber,const ADC_SocTrigSrc_e trigSrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCSOCxCTL[socNumber] &= (~ADC_ADCSOCxCTL_TRIGSEL_BITS);


  // set the bits
  adc->ADCSOCxCTL[socNumber] |= trigSrc;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSocTrigSrc() function


void ADC_setSocFrc(ADC_Handle adcHandle,const ADC_SocFrc_e socFrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCSOCFRC1 = 1 << socFrc;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSocFrc() function


void ADC_setSocFrcWord(ADC_Handle adcHandle,const uint16_t socFrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // write the entire word
  adc->ADCSOCFRC1 = socFrc;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setSocFrcWord() function


void ADC_setTempSensorSrc(ADC_Handle adcHandle,const ADC_TempSensorSrc_e sensorSrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_TEMPCONV_BITS);


  // set the bits
  adc->ADCCTL1 |= sensorSrc;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setTempSensorSrc() function


void ADC_setVoltRefSrc(ADC_Handle adcHandle,const ADC_VoltageRefSrc_e voltSrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_ADCREFSEL_BITS);


  // set the bits
  adc->ADCCTL1 |= voltSrc;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setVoltRefSrc() function


extern ADC_DivideSelect_e ADC_getDivideSelect(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;

  // get the bits
  ADC_DivideSelect_e divSelect = (ADC_DivideSelect_e)((adc->ADCCTL2) & (ADC_ADCCTL2_CLKDIV2EN_BITS));

  if(divSelect == 4)
    divSelect = ADC_DivideSelect_ClkIn_by_1;

  return (divSelect);
} // end of ADC_getDivideSelect() function


void ADC_setDivideSelect(ADC_Handle adcHandle,const ADC_DivideSelect_e divSelect)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL2 &= (~(ADC_ADCCTL2_CLKDIV2EN_BITS));


  // set the bits
  adc->ADCCTL2 |= divSelect;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setDivideSelect() function


void ADC_enableNoOverlapMode(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL2 |= ADC_ADCCTL2_ADCNONOVERLAP_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enableNoOverlapMode() function


void ADC_disableNoOverlapMode(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // clear the bits
  adc->ADCCTL2 &= (~(ADC_ADCCTL2_ADCNONOVERLAP_BITS));

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enableNoOverlapMode() function


void ADC_setupSocTrigSrc(ADC_Handle adcHandle, const ADC_SocNumber_e socNumber, const ADC_IntTriggerSOC_e intTrigSrc)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;
  uint16_t clearValue;
  uint16_t setValue;
  uint16_t lShift_socsel1 = socNumber << 1;
  uint16_t lShift_socsel2 = (socNumber - ADC_SocNumber_8) << 1;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(socNumber < ADC_SocNumber_8)
    {
      clearValue = ADC_ADCINTSOCSELx_SOCx_BITS << lShift_socsel1;
      setValue = intTrigSrc << lShift_socsel1;

      // clear the bits
      adc->ADCINTSOCSEL1 &= (~(clearValue));

      // set the bits
      adc->ADCINTSOCSEL1 |= setValue;
    }
  else
    {
      clearValue = ADC_ADCINTSOCSELx_SOCx_BITS << lShift_socsel2;
      setValue = intTrigSrc << lShift_socsel2;

      // clear the bits
      adc->ADCINTSOCSEL2 &= (~(clearValue));

      // set the bits
      adc->ADCINTSOCSEL2 |= setValue;
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setupSocTrigSrc() function

void ADC_setOffTrim(ADC_Handle adcHandle, const uint16_t offtrim)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the offtrim bits
  adc->ADCOFFTRIM = offtrim;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_setOffTrim() function

void ADC_enableVoltRefLoConv(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 |= ADC_ADCCTL1_VREFLOCONV_BITS;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_enableVoltRefLoConv() function

void ADC_disableVoltRefLoConv(ADC_Handle adcHandle)
{
  ADC_Obj *adc = (ADC_Obj *)adcHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // set the bits
  adc->ADCCTL1 &= (~ADC_ADCCTL1_VREFLOCONV_BITS);

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of ADC_disableVoltRefLoConv() function

// end of file
