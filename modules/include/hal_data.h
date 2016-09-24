#ifndef _HAL_DATA_H_
#define _HAL_DATA_H_

//! \file   ~/sw/modules/hal/src/32b/hal_data.h
//! \brief  Contains the HAL data structures
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// drivers


// modules
#include "IQmathLib.h"
#include "math.h"


// solutions


//!
//!
//! \defgroup HAL_DATA HAL_DATA
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines the ADC data
//!
typedef struct _HAL_AdcData_t_
{
  MATH_vec3  I_pu;          //!< the current values

  MATH_vec3  V_pu;          //!< the voltage values

  _iq        dcBus_pu;      //!< the dcBus value

} HAL_AdcData_t;


//! \brief Defines the DAC data
//!
typedef struct _HAL_DacData_t_
{
  _iq        value[4];      //!< the DAC data

} HAL_DacData_t;


//! \brief Defines the PWM data
//!
typedef struct _HAL_PwmData_t_
{
  MATH_vec3  Vabc_pu;      //!< the PWM time-durations for each motor phase

} HAL_PwmData_t;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup


#endif // end of _HAL_DATA_H_ definition

