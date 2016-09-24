#ifndef _EST_STATES_H_
#define _EST_STATES_H_

//! \file   ~/sw/modules/est/src/est_states.h
//! \brief  Contains the states for the estimator (EST) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_STATES EST_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the includes


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the estimator error codes
//!
typedef enum
{
  EST_ErrorCode_NoError=0,               //!< no error error code
  EST_ErrorCode_Flux_OL_ShiftOverFlow,   //!< flux open loop shift overflow error code
  EST_ErrorCode_FluxError,               //!< flux estimator error code
  EST_ErrorCode_Dir_ShiftOverFlow,       //!< direction shift overflow error code
  EST_ErrorCode_Ind_ShiftOverFlow,       //!< inductance shift overflow error code
  EST_numErrorCodes                      //!< the number of estimator error codes
} EST_ErrorCode_e;


//! \brief Enumeration for the estimator states
//!
typedef enum
{
  EST_State_Error=0,            //!< error
  EST_State_Idle,               //!< idle
  EST_State_RoverL,             //!< R/L estimation
  EST_State_Rs,                 //!< Rs estimation state
  EST_State_RampUp,             //!< ramp up the speed
#if !defined(FAST_ROM_V1p6) && !defined(FAST_ROM_V1p7)
  EST_State_ConstSpeed,         //!< constant speed after ramp up
#endif
  EST_State_IdRated,            //!< control Id and estimate the rated flux
  EST_State_RatedFlux_OL,       //!< estimate the open loop rated flux
  EST_State_RatedFlux,          //!< estimate the rated flux 
  EST_State_RampDown,           //!< ramp down the speed 
  EST_State_LockRotor,          //!< lock the rotor
  EST_State_Ls,                 //!< stator inductance estimation state
  EST_State_Rr,                 //!< rotor resistance estimation state
  EST_State_MotorIdentified,    //!< motor identified state
  EST_State_OnLine,             //!< online parameter estimation
  EST_numStates                 //!< the number of estimator states
} EST_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_STATES_H_ definition

