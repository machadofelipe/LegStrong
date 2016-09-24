#ifndef _EST_FLUX_STATES_H_
#define _EST_FLUX_STATES_H_

//! \file   ~/sw/modules/est/src/est_Flux_states.h
//! \brief  Contains the states for the flux 
//!         estimator (EST_Flux) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_FLUX_STATES EST_FLUX_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the estimator error codes
//!
typedef enum
{
  EST_Flux_ErrorCode_NoError=0,      //!< no error error code
  EST_Flux_ErrorCode_ShiftOverFlow,  //!< flux shift overflow error code
  EST_Flux_ErrorCode_Clip,           //!< flux clip error code
  EST_Flux_numErrorCodes             //!< the number of estimator error codes
} EST_Flux_ErrorCode_e;


//! \brief Enumeration for the estimator states
//!
typedef enum
{
  EST_Flux_State_Error=0,  //!< error state
  EST_Flux_State_Idle,     //!< idle state
  EST_Flux_State_CL1,      //!< closed loop control stage 1
  EST_Flux_State_CL2,      //!< closed loop control stage 2
  EST_Flux_State_Fine,     //!< fine estimate of flux
  EST_Flux_State_Done,     //!< done state
  EST_Flux_numStates       //!< the number of flux estimator states
} EST_Flux_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_FLUX_STATES_H_ definition

