#ifndef _EST_RS_STATES_H_
#define _EST_RS_STATES_H_

//! \file   ~/sw/modules/est/src/est_Rs_states.h
//! \brief  Contains the public interface to the stator 
//!         resistance estimator (EST_Rs) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_RS_STATES EST_RS_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the stator resistance estimator states
//!
typedef enum
{
  EST_Rs_State_Error=0,     //!< error
  EST_Rs_State_Idle,        //!< idle
  EST_Rs_State_RampUp,      //!< the Id ramp up state
  EST_Rs_State_Coarse,      //!< the coarse estimation state
  EST_Rs_State_Fine,        //!< the fine estimation state
  EST_Rs_State_Done,        //!< the done state
  EST_Rs_numStates          //!< the number of stator resistance estimator states
} EST_Rs_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_RS_STATES_H_ definition

