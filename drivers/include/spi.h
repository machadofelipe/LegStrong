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
#ifndef _SPI_H_
#define _SPI_H_

//! \file   drivers/spi/src/32b/f28x/f2802x/spi.h
//! \brief  Contains public interface to various functions related
//!         to the serial peripheral interface (SPI) object
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
//! \defgroup SPI SPI
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the base address of the serial peripheral interface (SPI) A registers
//!
#define SPIA_BASE_ADDR              (0x00007040)


//! \brief Defines the location of the SPICHAR3-0 bits in the SPICCR register
//!
#define SPI_SPICCR_CHAR_LENGTH_BITS        (15 << 0)

//! \brief Defines the location of the SPILBK bits in the SPICCR register
//!
#define SPI_SPICCR_SPILBK_BITS             ( 1 << 4)  

//! \brief Defines the location of the CLOCK POLARITY bits in the SPICCR register
//!
#define SPI_SPICCR_CLKPOL_BITS             ( 1 << 6)

//! \brief Defines the location of the SPI SW Reset bits in the SPICCR register
//!
#define SPI_SPICCR_RESET_BITS              ( 1 << 7)

//! \brief Defines the location of the SPI INT Flag in the SPIST register
//!
#define SPI_SPIST_INTFLAG_BITS              ( 1 << 6)

//! \brief Defines the location of the TX BUF FULL FLAG in the SPIST register
//!
#define SPI_SPIST_TXBUF_BITS                (1 << 5)

//! \brief Defines the location of the SPI INT ENA bits in the SPICTL register
//!
#define SPI_SPICTL_INT_ENA_BITS             (1 << 0)

//! \brief Defines the location of the TALK bits in the SPICTL register
//!
#define SPI_SPICTL_TALK_BITS                (1 << 1)

//! \brief Defines the location of the MASTER/SLAVE bits in the SPICTL register
//!
#define SPI_SPICTL_MODE_BITS                (1 << 2)

//! \brief Defines the location of the CLOCK PHASE bits in the SPICTL register
//!
#define SPI_SPICTL_CLK_PHASE_BITS           (1 << 3)

//! \brief Defines the location of the OVERRUN INT ENA bits in the SPICTL register
//!
#define SPI_SPICTL_OVRRUN_INT_ENA_BITS      (1 << 4)


//! \brief Defines the location of the RXFFIL4-0 bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_IL_BITS           (31 << 0)

//! \brief Defines the location of the RXFFIENA bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_IENA_BITS         ( 1 << 5)

//! \brief Defines the location of the RXFFINT CLR bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_INTCLR_BITS       ( 1 << 6)

//! \brief Defines the location of the RXFFINT CLR bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_INT_BITS          ( 1 << 7)

//! \brief Defines the location of the RXFFST4-0 bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_ST_BITS      (31 << 8)

//! \brief Defines the location of the RXFIFO Reset bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_RESET_BITS   ( 1 << 13)

//! \brief Defines the location of the RXFFOVF CLR bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_OVFCLR_BITS  ( 1 << 14)

//! \brief Defines the location of the RXFFOVF bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_OVF_BITS     ( 1 << 15)


//! \brief Defines the location of the TXFFIL4-0 bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_IL_BITS           (31 << 0)

//! \brief Defines the location of the TXFFIENA bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_IENA_BITS         ( 1 << 5)

//! \brief Defines the location of the TXFFINT CLR bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_INTCLR_BITS       ( 1 << 6)

//! \brief Defines the location of the TXFFINT bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_INT_BITS          ( 1 << 7)

//! \brief Defines the location of the TXFFST4-0 bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_FIFO_ST_BITS      (31 << 8)

//! \brief Defines the location of the TXFIFO Reset bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_FIFO_RESET_BITS   ( 1 << 13)

//! \brief Defines the location of the SPIFFENA bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_FIFO_ENA_BITS     ( 1 << 14)

//! \brief Defines the location of the SPIRST bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_CHAN_RESET_BITS   ( 1 << 15)


//! \brief Defines the location of the SUSP bits in the SPIPRI register
//!
#define SPI_SPIPRI_SUSP_BITS          (  3 << 4)

//! \brief Defines the location of the STE_INV bits in the SPIPRI register
//!
#define SPI_SPIPRI_STE_INV_BITS       (  1 << 1)

//! \brief Defines the location of the TRIWIRE bits in the SPIPRI register
//!
#define SPI_SPIPRI_TRIWIRE            (  1 << 0)


// **************************************************************************
// the typedefs

//! \brief Enumeration to define the serial peripheral interface (SPI) baud rates
//!
typedef enum
{
  SPI_BaudRate_LSPCLK_Over_Four=0,
  SPI_BaudRate_1_MBaud=(14 << 0)   //!< Denotes 1 MBaud
} SPI_BaudRate_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) character lengths
//!
typedef enum
{
  SPI_CharLength_1_Bit=(0 << 0),        //!< Denotes a character length of 1 bit
  SPI_CharLength_2_Bits=(1 << 0),       //!< Denotes a character length of 2 bits
  SPI_CharLength_3_Bits=(2 << 0),       //!< Denotes a character length of 3 bits
  SPI_CharLength_4_Bits=(3 << 0),       //!< Denotes a character length of 4 bits
  SPI_CharLength_5_Bits=(4 << 0),       //!< Denotes a character length of 5 bits
  SPI_CharLength_6_Bits=(5 << 0),       //!< Denotes a character length of 6 bits
  SPI_CharLength_7_Bits=(6 << 0),       //!< Denotes a character length of 7 bits
  SPI_CharLength_8_Bits=(7 << 0),       //!< Denotes a character length of 8 bits
  SPI_CharLength_9_Bits=(8 << 0),       //!< Denotes a character length of 9 bits
  SPI_CharLength_10_Bits=(9 << 0),      //!< Denotes a character length of 10 bits
  SPI_CharLength_11_Bits=(10 << 0),     //!< Denotes a character length of 11 bits
  SPI_CharLength_12_Bits=(11 << 0),     //!< Denotes a character length of 12 bits
  SPI_CharLength_13_Bits=(12 << 0),     //!< Denotes a character length of 13 bits
  SPI_CharLength_14_Bits=(13 << 0),     //!< Denotes a character length of 14 bits
  SPI_CharLength_15_Bits=(14 << 0),     //!< Denotes a character length of 15 bits
  SPI_CharLength_16_Bits=(15 << 0)      //!< Denotes a character length of 16 bits
} SPI_CharLength_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) clock phase
//!
typedef enum
{
  SPI_ClkPhase_Normal=(0<<3),      //!< Denotes a normal clock scheme
  SPI_ClkPhase_Delayed=(1<<3)      //!< Denotes that the SPICLK signal is delayed by one half-cycle
} SPI_ClkPhase_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) clock polarity for the input and output data
//!
typedef enum
{
  SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge=(0 << 6),  //!< Denotes that the tx data is output on the rising edge, the rx data is latched on the falling edge
  SPI_ClkPolarity_OutputFallingEdge_InputRisingEdge=(1 << 6)   //!< Denotes that the tx data is output on the falling edge, the rx data is latched on the rising edge 
} SPI_ClkPolarity_e;



//! \brief Enumeration to define the serial peripheral interface (SPI) network mode control
//!
typedef enum
{
  SPI_Mode_Slave=(0<<2),      //!< Denotes slave mode
  SPI_Mode_Master=(1<<2)      //!< Denotes master mode
} SPI_Mode_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) FIFO level
//!
typedef enum
{
  SPI_FifoLevel_Empty=(0 << 0),      //!< Denotes the fifo is empty
  SPI_FifoLevel_1_Word=(1 << 0),     //!< Denotes the fifo contains 1 word
  SPI_FifoLevel_2_Words=(2 << 0),    //!< Denotes the fifo contains 2 words
  SPI_FifoLevel_3_Words=(3 << 0),    //!< Denotes the fifo contains 3 words
  SPI_FifoLevel_4_Words=(4 << 0)     //!< Denotes the fifo contains 4 words
} SPI_FifoLevel_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) FIFO status
//!
typedef enum
{
  SPI_FifoStatus_Empty=(0 << 8),      //!< Denotes the fifo is empty
  SPI_FifoStatus_1_Word=(1 << 8),     //!< Denotes the fifo contains 1 word
  SPI_FifoStatus_2_Words=(2 << 8),    //!< Denotes the fifo contains 2 words
  SPI_FifoStatus_3_Words=(3 << 8),    //!< Denotes the fifo contains 3 words
  SPI_FifoStatus_4_Words=(4 << 8)     //!< Denotes the fifo contains 4 words
}  SPI_FifoStatus_e;


//! \brief Enumeration to define the the serial peripheral interface (SPI) priority
//!
typedef enum
{
  SPI_Priority_Immediate=(0 << 4),      //!< Stops immediately after EMU halt
  SPI_Priority_FreeRun=(1 << 4),        //!< Doesn't stop after EMU halt
  SPI_Priority_AfterRxRxSeq=(2 << 4)    //!< Stops after EMU halt and next rx/rx sequence
} SPI_Priority_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) Interrupt Flag Status
//!
typedef enum
{
  SPI_IntFlagStatus_InProgress=(0<<6),  //!< Denotes transmission or reception in progress
  SPI_IntFlagStatus_Completed=(1<<6)    //!< Denotes transmission or reception completed
} SPI_IntFlagStatus_e;


//! \brief Enumeration to define the the serial peripheral interface (SPI) STE pin status
//!
typedef enum
{
  SPI_SteInv_ActiveLow=(0 << 1),        //!< Denotes active low STE pin
  SPI_SteInv_ActiveHigh=(1 << 1)        //!< Denotes active high STE pin
} SPI_SteInv_e;


//! \brief Enumeration to define the tri-wire status
//!
typedef enum
{
  SPI_TriWire_NormalFourWire=(0 << 0),  //!< Denotes 4 wire SPI mode
  SPI_TriWire_ThreeWire=(1 << 0)        //!< Denotes 3 wire SPI mode
} SPI_TriWire_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) Tx Buffer Status
//!
typedef enum
{
  SPI_TxBufferStatus_Empty=(0<<5),   //!< Denotes that the Tx buffer is empty
  SPI_TxBufferStatus_Full=(1<<5)     //!< Denotes that the Tx buffer is full
} SPI_TxBufferStatus_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) Enumeration suspend bits
//!
typedef enum
{
  SPI_TxSuspend_00=(0x0<<4),      //!< Emulation Suspend option 1
  SPI_TxSuspend_10=(0x2<<4),      //!< Emulation Suspend option 2
  SPI_TxSuspend_free=(0x1<<4)     //!< Emulation Free run
} SPI_EmulationSuspend_e;


//! \brief Defines the serial peripheral interface (SPI) object
//!
typedef struct _SPI_Obj_
{
  volatile uint16_t      SPICCR;        //!< SPI Configuration Control Register
  volatile uint16_t      SPICTL;        //!< SPI Operation Control Register
  volatile uint16_t      SPIST;         //!< SPI Status Register
  volatile uint16_t      rsvd_1;        //!< Reserved
  volatile uint16_t      SPIBRR;        //!< SPI Baud Rate Register
  volatile uint16_t      rsvd_2;        //!< Reserved
  volatile uint16_t      SPIEMU;        //!< SPI Emulation Buffer Register
  volatile uint16_t      SPIRXBUF;      //!< SPI Serial Input Buffer Register
  volatile uint16_t      SPITXBUF;      //!< SPI Serial Output Buffer Register
  volatile uint16_t      SPIDAT;        //!< SPI Serial Data Register
  volatile uint16_t      SPIFFTX;       //!< SPI FIFO Transmit Register
  volatile uint16_t      SPIFFRX;       //!< SPI FIFO Receive Register
  volatile uint16_t      SPIFFCT;       //!< SPI FIFO Control Register
  volatile uint16_t      rsvd_3[2];     //!< Reserved
  volatile uint16_t      SPIPRI;        //!< SPI Priority Register
} SPI_Obj;


//! \brief Defines the serial peripheral interface (SPI) handle
//!
typedef struct _SPI_Obj_ *SPI_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Clears the Rx FIFO overflow flag
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_clearRxFifoOvf(SPI_Handle spiHandle);


//! \brief     Clears the Rx FIFO interrupt flag
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_clearRxFifoInt(SPI_Handle spiHandle);


//! \brief     Clears the Tx FIFO interrupt flag
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_clearTxFifoInt(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableInt(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) loop back mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableLoopBack(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) over-run interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableOverRunInt(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) receive FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableRxFifoInt(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) master/slave transmit mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableTx(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) transmit FIFO enhancements
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableTxFifoEnh(SPI_Handle spiHandle);


//! \brief     Disables the serial peripheral interface (SPI) transmit FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableTxFifoInt(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enable(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) transmit and receive channels
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableChannels(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableInt(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) loop back mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableLoopBack(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) over-run interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableOverRunInt(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) receive FIFO
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableRxFifo(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) receive FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableRxFifoInt(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) masater/slave transmit mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTx(SPI_Handle spiHandle);


//! \brief     Re-enables the serial peripheral interface (SPI) transmit FIFO
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTxFifo(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) transmit FIFO enhancements
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTxFifoEnh(SPI_Handle spiHandle);


//! \brief     Enables the serial peripheral interface (SPI) transmit FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTxFifoInt(SPI_Handle spiHandle);


//! \brief     Gets the serial peripheral interface (SPI) receive FIFO status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The receive FIFO status
extern SPI_FifoStatus_e SPI_getRxFifoStatus(SPI_Handle spiHandle);


//! \brief     Gets the serial peripheral interface (SPI) transmit FIFO status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The transmit FIFO status
extern SPI_FifoStatus_e SPI_getTxFifoStatus(SPI_Handle spiHandle);


//! \brief     Gets the serial peripheral interface (SPI) Interrupt Flag status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The Interrupt Flag status
extern SPI_IntFlagStatus_e SPI_getIntFlagStatus(SPI_Handle spiHandle);


//! \brief     Gets the serial peripheral interface (SPI) Tx Buffer status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The Interrupt Flag status
extern SPI_TxBufferStatus_e SPI_getTxBufferStatus(SPI_Handle spiHandle);


//! \brief     Initializes the serial peripheral interface (SPI) object handle
//! \param[in] pMemory     A pointer to the base address of the SPI registers
//! \param[in] numBytes    The number of bytes allocated for the SPI object, bytes
//! \return    The serial peripheral interface (SPI) object handle
extern SPI_Handle SPI_init(void *pMemory,const size_t numBytes);


//! \brief     Reads data from the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The received data value
static inline uint16_t SPI_read(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // read the data
  uint16_t data = spi->SPIRXBUF;

  return(data);
} // end of SPI_read() function


//! \brief     Reads data from the serial peripheral interface (SPI) during emulation
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The received data value
static inline uint16_t SPI_readEmu(SPI_Handle spiHandle)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // read the data
  uint16_t data = spi->SPIEMU;

  return(data);
} // end of SPI_readEmu() function


//! \brief     Resets the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_reset(SPI_Handle spiHandle);


//! \brief     Resets the serial peripheral interface (SPI) transmit and receive channels
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_resetChannels(SPI_Handle spiHandle);


//! \brief     Resets the serial peripheral interface (SPI) receive FIFO
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_resetRxFifo(SPI_Handle spiHandle);


//! \brief     Resets the serial peripheral interface (SPI) transmit FIFO
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_resetTxFifo(SPI_Handle spiHandle);


//! \brief     Sets the serial peripheral interface (SPI) baud rate
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] baudRate   The baud rate
extern void SPI_setBaudRate(SPI_Handle spiHandle,const SPI_BaudRate_e baudRate);


//! \brief     Sets the serial peripheral interface (SPI) character length
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] length     The character length
extern void SPI_setCharLength(SPI_Handle spiHandle,const SPI_CharLength_e length);


//! \brief     Sets the serial peripheral interface (SPI) clock phase
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] clkPhase   The clock phase
extern void SPI_setClkPhase(SPI_Handle spiHandle,const SPI_ClkPhase_e clkPhase);


//! \brief     Sets the serial peripheral interface (SPI) clock polarity
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] polarity   The clock polarity
extern void SPI_setClkPolarity(SPI_Handle spiHandle,const SPI_ClkPolarity_e polarity);


//! \brief     Sets the serial peripheral interface (SPI) network mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] mode       The network mode
extern void SPI_setMode(SPI_Handle spiHandle,const SPI_Mode_e mode);


//! \brief     Sets the serial peripheral interface (SPI) receive FIFO level for generating an interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] fifoLevel  The FIFO level
extern void SPI_setRxFifoIntLevel(SPI_Handle spiHandle,const SPI_FifoLevel_e fifoLevel);


//! \brief     Sets the priority of the SPI port vis-a-vis the EMU
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] priority   The priority of the SPI port vis-a-vis the EMU
extern void SPI_setPriority(SPI_Handle spiHandle,const SPI_Priority_e priority);


//! \brief     Controls pin inversion of STE pin 
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] steinv     Polarity of STE pin
extern void SPI_setSteInv(SPI_Handle spiHandle,const SPI_SteInv_e steinv);


//! \brief     Sets the serial peripheral interface (SPI) emulation suspend bits
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] SPI_EmulationSuspend_e The emulation suspend enumeration
extern void SPI_setSuspend(SPI_Handle spiHandle,const SPI_EmulationSuspend_e emuSuspend);


//! \brief     Sets SPI port operating mode 
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] triwire    3 or 4 wire mode
extern void SPI_setTriWire(SPI_Handle spiHandle,const SPI_TriWire_e triwire);


//! \brief     Sets the serial peripheral interface (SPI) transmit delay
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] delay      The delay value
extern void SPI_setTxDelay(SPI_Handle spiHandle,const uint_least8_t delay);


//! \brief     Sets the serial peripheral interface (SPI) transmit FIFO level for generating an interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] fifoLevel  The FIFO level
extern void SPI_setTxFifoIntLevel(SPI_Handle spiHandle,const SPI_FifoLevel_e fifoLevel);


//! \brief     Writes data to the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] data       The data value
static inline void SPI_write(SPI_Handle spiHandle,const uint16_t data)
{
  SPI_Obj *spi = (SPI_Obj *)spiHandle;


  // set the bits
  spi->SPITXBUF = data;

  return;
} // end of SPI_write() function


//! \brief     Writes a byte of data to the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] data       The data value
static inline void SPI_write8(SPI_Handle spiHandle,const uint16_t data)
{
    SPI_Obj *spi = (SPI_Obj *)spiHandle;


    // set the bits
    spi->SPITXBUF = (data & 0xFF) << 8;

    return;
} // end of SPI_write() function

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _SPI_H_ definition

