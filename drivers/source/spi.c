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
//! \file   drivers/spi/src/32b/f28x/f2802x/spi.c
//! \brief  Contains the various functions related to the 
//!         serial peripheral interface (SPI) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "spi.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void SPI_clearRxFifoOvf(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFRX |= SPI_SPIFFRX_FIFO_OVFCLR_BITS;

  return;
} // end of SPI_clearRxFifoOvf() function


void SPI_clearRxFifoInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFRX |= SPI_SPIFFRX_INTCLR_BITS;

  return;
} // end of SPI_clearRxFifoInt() function


void SPI_clearTxFifoInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFTX |= SPI_SPIFFTX_INTCLR_BITS;

  return;
} // end of SPI_clearTxFifoInt() function


void SPI_disableInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPICTL &= (~SPI_SPICTL_INT_ENA_BITS);

  return;
} // end of SPI_disableInt() function


void SPI_disableLoopBack(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPICCR &= (~SPI_SPICCR_SPILBK_BITS);

  return;
} // end of SPI_disableLoopBack() function


void SPI_disableOverRunInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;

  // clear the bits
  spi->SPICTL &= (~SPI_SPICTL_OVRRUN_INT_ENA_BITS);

  return;
} // end of SPI_disableOverRunInt() function


void SPI_disableRxFifoInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPIFFRX &= (~SPI_SPIFFRX_IENA_BITS);

  return;
} // end of SPI_disableRxFifoInt() function


void SPI_disableTx(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPICTL &= (~SPI_SPICTL_TALK_BITS);

  return;
} // end of SPI_disableTx() function


void SPI_disableTxFifoEnh(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPIFFTX &= (~SPI_SPIFFTX_FIFO_ENA_BITS);

  return;
} // end of SPI_disableTxFifo() function


void SPI_disableTxFifoInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPIFFTX &= (~SPI_SPIFFTX_IENA_BITS);

  return;
} // end of SPI_disableTxFifoInt() function


void SPI_enable(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPICCR |= SPI_SPICCR_RESET_BITS;

  return;
} // end of SPI_enable() function


void SPI_enableChannels(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFTX |= (uint16_t)SPI_SPIFFTX_CHAN_RESET_BITS;
  
  return;
} // SPI_enableChannels() function


void SPI_enableInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPICTL |= SPI_SPICTL_INT_ENA_BITS;

  return;
} // end of SPI_enableInt() function


void SPI_enableLoopBack(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPICCR |= SPI_SPICCR_SPILBK_BITS;

  return;
} // end of SPI_enableLoopBack() function


void SPI_enableOverRunInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPICTL |= SPI_SPICTL_OVRRUN_INT_ENA_BITS;

  return;
} // end of SPI_enableOverRunInt() function


void SPI_enableRxFifo(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFRX |= SPI_SPIFFRX_FIFO_RESET_BITS;

  return;
} // end of SPI_enableRxFifo() function


void SPI_enableRxFifoInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFRX |= SPI_SPIFFRX_IENA_BITS;

  return;
} // end of SPI_enableRxFifoInt() function


void SPI_enableTx(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPICTL |= SPI_SPICTL_TALK_BITS;

  return;
} // end of SPI_enableTx() function


void SPI_enableTxFifo(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFTX |= SPI_SPIFFTX_FIFO_RESET_BITS;

  return;
} // end of SPI_enableTxFifo() function


void SPI_enableTxFifoEnh(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFTX |= SPI_SPIFFTX_FIFO_ENA_BITS;

  return;
} // end of SPI_enableTxFifo() function


void SPI_enableTxFifoInt(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFTX |= SPI_SPIFFTX_IENA_BITS;

  return;
} // end of SPI_enableTxFifoInt() function


SPI_FifoStatus_e SPI_getRxFifoStatus(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // get the status
  SPI_FifoStatus_e status = (SPI_FifoStatus_e)((spi->SPIFFRX) & SPI_SPIFFRX_FIFO_ST_BITS);

  return(status);
} // SPI_getRxFifoStatus() function


SPI_FifoStatus_e SPI_getTxFifoStatus(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // get the status
  SPI_FifoStatus_e status = (SPI_FifoStatus_e)(spi->SPIFFTX & SPI_SPIFFTX_FIFO_ST_BITS);

  return(status);
} // SPI_getTxFifoStatus() function


SPI_IntFlagStatus_e SPI_getIntFlagStatus(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // get the status
  SPI_IntFlagStatus_e status = (SPI_IntFlagStatus_e)(spi->SPIST & SPI_SPIST_INTFLAG_BITS);

  return(status);
} // SPI_getIntFlagStatus() function


SPI_TxBufferStatus_e SPI_getTxBufferStatus(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // get the status
  SPI_TxBufferStatus_e status = (SPI_TxBufferStatus_e)(spi->SPIST & SPI_SPIST_TXBUF_BITS);

  return(status);
} // SPI_getTxBufferStatus() function


SPI_Handle SPI_init(void *pMemory,const size_t numBytes)
{
  SPI_Handle spiHandle;


  if(numBytes < sizeof(SPI_Obj))
    return((SPI_Handle)NULL);

  // assign the handle
  spiHandle = (SPI_Handle)pMemory;
  
  return(spiHandle);
} // end of SPI_init() function


void SPI_reset(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPICCR &= (~SPI_SPICCR_RESET_BITS);

  return;
} // end of SPI_reset() function


void SPI_resetChannels(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPIFFTX &= (~SPI_SPIFFTX_CHAN_RESET_BITS);
  
  return;
} // SPI_resetChannels() function


void SPI_resetRxFifo(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPIFFRX &= (~SPI_SPIFFRX_FIFO_RESET_BITS);

  return;
} // end of SPI_resetRxFifo() function


void SPI_resetTxFifo(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPIFFTX &= (~SPI_SPIFFTX_FIFO_RESET_BITS);

  return;
} // end of SPI_resetTxFifo() function


void SPI_setBaudRate(SPI_Handle spiHandle,const SPI_BaudRate_e baudRate)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;

  // set the bits
  spi->SPIBRR = baudRate;
  return;
} // end of SPI_setBaudRate() function


void SPI_setCharLength(SPI_Handle spiHandle,const SPI_CharLength_e length)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPICCR &= (~SPI_SPICCR_CHAR_LENGTH_BITS);

  // set the bits
  spi->SPICCR |= length;

  return;
} // end of SPI_setCharLength() function


void SPI_setClkPhase(SPI_Handle spiHandle,const SPI_ClkPhase_e clkPhase)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the value
  spi->SPICTL |= clkPhase;

  return;
} // end of SPI_setClkPhase() function


void SPI_setClkPolarity(SPI_Handle spiHandle,const SPI_ClkPolarity_e polarity)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the bits
  spi->SPICCR &= (~SPI_SPICCR_CLKPOL_BITS);

  // set the bits
  spi->SPICCR |= polarity;

  return;
} // end of SPI_setClkPolarity() function


void SPI_setMode(SPI_Handle spiHandle,const SPI_Mode_e mode)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPICTL |= mode;

  return;
} // end of SPI_setMode() function


void SPI_setPriority(SPI_Handle spiHandle,const SPI_Priority_e priority)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the field
  spi->SPIPRI &= (~SPI_SPIPRI_SUSP_BITS);
  // set the bits
  spi->SPIPRI |= priority;

  return;
} // SPI_setPriority() function


void SPI_setRxFifoIntLevel(SPI_Handle spiHandle,const SPI_FifoLevel_e fifoLevel)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the value
  spi->SPIFFRX &= (~SPI_SPIFFRX_IL_BITS);

  // set the bits
  spi->SPIFFRX |= fifoLevel;

  return;
} // end of SPI_setRxFifoIntLevel() function


void SPI_setSteInv(SPI_Handle spiHandle,const SPI_SteInv_e steinv)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;

  // clear the field
  spi->SPIPRI &= (~SPI_SPIPRI_STE_INV_BITS);
  // set the bits
  spi->SPIPRI |= steinv;

  return;
} // SPI_setSteInv() function


void SPI_setSuspend(SPI_Handle spiHandle,const SPI_EmulationSuspend_e emuSuspend)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;

  // clear the field
  spi->SPIPRI &= (~SPI_SPIPRI_SUSP_BITS);

  // set the bits
  spi->SPIPRI |= emuSuspend;

  return;
} // SPI_setSuspend() function


void SPI_setTriWire(SPI_Handle spiHandle,const SPI_TriWire_e triwire)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;

  // clear the field
  spi->SPIPRI &= (~SPI_SPIPRI_TRIWIRE);
  // set the bits
  spi->SPIPRI |= triwire;

  return;
} // SPI_setTriWire() function


void SPI_setTxDelay(SPI_Handle spiHandle,const uint_least8_t delay)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPIFFCT = delay;

  return;
} // end of SPI_setTxDelay() function


void SPI_setTxFifoIntLevel(SPI_Handle spiHandle,const SPI_FifoLevel_e fifoLevel)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // clear the value
  spi->SPIFFTX &= (~SPI_SPIFFTX_IL_BITS);

  // set the bits
  spi->SPIFFTX |= fifoLevel;

  return;
} // end of SPI_setTxFifoIntLevel() function


// end of file
