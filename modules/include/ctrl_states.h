#ifndef _CTRL_STATES_H_
#define _CTRL_STATES_H_

//! \file   ~/dmc_mw/sw/modules/fast/src/ctrl_states.h
//! \brief  Contains the states for the controller (CTRL) object
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup CTRL_STATES CTRL_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the controller states
//!
typedef enum {
  CTRL_State_Error=0,           //!< the controller error state
  CTRL_State_Idle,              //!< the controller idle state
  CTRL_State_OffLine,           //!< the controller offline state
  CTRL_State_OnLine,            //!< the controller online state
  CTRL_numStates                //!< the number of controller states
} CTRL_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CTRL_STATES_H_ definition

