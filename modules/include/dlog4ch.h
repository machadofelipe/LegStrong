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
#ifndef _DLOG4CH_H_
#define _DLOG4CH_H_

//! \file   modules/dlog/src/32b/dlog4ch.h
//! \brief  Contains the public interface to the 
//!         data logging (DLOG) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "types.h"



//!
//!
//! \defgroup DLOG4CH DLOG4CH
//!
//@{
// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup DLOG_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the default initalization for the DLOG object
//!
                     
#define DLOG_4CH_DEFAULTS { 0UL, \
                            NULL, \
                            NULL, \
                            NULL, \
                            NULL, \
                            0, \
                            1, \
                            0, \
                            0, \
                            0UL, \
                            0x0C0, \
                            (int16_t (*)(int))DLOG_4CH_init, \
                            (int16_t (*)(int))DLOG_4CH_update }



// **************************************************************************
// the typedefs
 
//! \brief Defines the data logging (DLOG) object
//!
typedef struct _DLOG_4CH_
{    
  int32_t  task;          //!< Variable:  Task address pointer   
  int16_t  *iptr1;        //!< Input: First input pointer
  int16_t  *iptr2;        //!< Input: Second input pointer
  int16_t  *iptr3;        //!< Input: Third input pointer
  int16_t  *iptr4;        //!< Input: Fourth input pointer
  int16_t  trig_value;    //!< Input: Trigger point
  int16_t  prescalar;     //!< Parameter: Data log prescale      
  int16_t  skip_cntr;     //!< Variable:  Data log skip counter      
  int16_t  cntr;          //!< Variable:  Data log counter       
  int32_t  write_ptr;     //!< Variable:  Graph address pointer               
  int16_t  size;          //!< Parameter: Maximum data buffer     
  int16_t (*init)();      //!< Pointer to init function          
  int16_t (*update)();    //!< Pointer to update function         
} DLOG_4CH;                
                                                         

//! \brief Defines the DLOG handle
//!
typedef struct _DLOG_4CH_Obj_   *DLOG_4CH_handle;
                                                         

// **************************************************************************
// the globals


//! \brief Defines the DLOG object
//!
extern DLOG_4CH dlog;


//! \brief Defines DLOG channel 1
//!
extern int16_t DlogCh1;

//! \brief Defines DLOG channel 2
//!
extern int16_t DlogCh2;

//! \brief Defines DLOG channel 3
//!
extern int16_t DlogCh3;

//! \brief Defines DLOG channel 4
//!
extern int16_t DlogCh4;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Initializes the data logger
//! \param[in] ptr  The pointer to memory
extern void DLOG_4CH_init(void *ptr);


//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
extern void DLOG_4CH_update(void *ptr);     
                                                    

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _DLOG4CH_H_ definition

