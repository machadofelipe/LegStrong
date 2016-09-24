#ifndef _EST_TRAJ_STATES_H_
#define _EST_TRAJ_STATES_H_

//! \file   ~sw/modules/est/src/est_Traj_states.h
//! \brief  Contains the states for the flux 
//!         estimator (EST_Flux) module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_TRAJ EST_TRAJ
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the trajectory generator error codes
//!
typedef enum
{
  EST_Traj_ErrorCode_NoError=0,        //!< no error error code
  EST_Traj_ErrorCode_IdClip,           //!< Id clip error code
  EST_Traj_numErrorCodes               //!< the number of trajectory generator error codes
} EST_Traj_ErrorCode_e;


//! \brief Enumeration for the trajectory generator states
//!
typedef enum
{
  EST_Traj_State_Error=0,             //!< the trajectory generator error state
  EST_Traj_State_Idle,                //!< the trajectory generator idle state
  EST_Traj_State_Est,                 //!< the trajectory generator parameter estimation state
  EST_Traj_State_OnLine,              //!< the trajectory generator online state
  EST_Traj_numStates                  //!< the number of trajectory generator states
} EST_Traj_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_TRAJ_STATES_H_ definition

