#ifndef _EST_LS_STATES_H_
#define _EST_LS_STATES_H_

//! \file   ~/sw/modules/est/src/est_Ls_states.h
//! \brief  Contains the states for the stator 
//!         inductance estimator (EST_Ls) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes


//!
//!
//! \defgroup EST_LS_STATES EST_LS_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

//! \brief Enumeration for the stator inductance estimator error codes
//!
typedef enum
{
  EST_Ls_ErrorCode_NoError=0,       //!< no error error code
  EST_Ls_ErrorCode_ShiftOverFlow,   //!< inductance shift overflow error code
  EST_Ls_numErrorCodes              //!< the number of estimator error codes
} EST_Ls_ErrorCode_e;


//! \brief Enumeration for the stator inductance estimator states
//!
typedef enum
{
  EST_Ls_State_Error=0,     //!< error
  EST_Ls_State_Idle,        //!< idle
  EST_Ls_State_RampUp,      //!< the ramp up state
  EST_Ls_State_Init,        //!< the init state
  EST_Ls_State_Coarse,      //!< the coarse estimation state
  EST_Ls_State_Fine,        //!< the fine estimation state
  EST_Ls_State_Done,        //!< the done state
  EST_Ls_numStates          //!< the number of stator inductance estimator states
} EST_Ls_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_LS_STATES_H_ definition

