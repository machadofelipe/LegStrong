#ifndef _EST_RR_STATES_H_
#define _EST_RR_STATES_H_

//! \file   ~/sw/modules/est/src/est_Rr_states.h
//! \brief  Contains the public interface to the rotor 
//!         resistance estimator (EST_Rr) module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_RR EST_RR
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the rotor resistance estimator states
//!
typedef enum {
  EST_Rr_State_Error=0,         //!< error
  EST_Rr_State_Idle,	        //!< idle
  EST_Rr_State_RampUp,	        //!< the Id ramp up state
  EST_Rr_State_Coarse,	        //!< the coarse estimation state
  EST_Rr_State_Fine,	        //!< the fine estimation state
  EST_Rr_State_Done,            //!< the done state
  EST_Rr_numStates              //!< the number of stator resistance estimator states
} EST_Rr_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_RR_STATES_H_ definition

