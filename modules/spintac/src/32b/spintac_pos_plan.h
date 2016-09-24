#ifndef __SPINTAC_POS_PLAN_H__
#define __SPINTAC_POS_PLAN_H__
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
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
 * *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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

//! \file    modules/spintac/src/32b/spintac_pos_plan.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Position Plan component
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACPOSPLAN SpinTAC Position Plan
//@{

#include "spintac_version.h"

//! \brief Defines the amount of memory that needs to be allocated for each component
//! \brief Each configured Action takes 3 double words of memory
#define ST_POS_PLAN_ACT_DWORDS          (5)
//! \brief Each configured Condition takes 3 double words of memory
#define ST_POS_PLAN_COND_DWORDS         (3)
//! \brief Each configured Variable takes 2 double words of memory
#define ST_POS_PLAN_VAR_DWORDS          (2)
//! \brief Each configured Transition takes 7 double words of memory
#define ST_POS_PLAN_TRAN_DWORDS         (7)
//! \brief Each configured State takes 7 double words of memory
#define ST_POS_PLAN_STATE_DWORDS        (7)


#ifndef __ST_PLAN_ENUM__
#define __ST_PLAN_ENUM__
//! \brief Enumeration for the Plan Status States
//!
typedef enum {
  ST_PLAN_IDLE=0, //!< idle state, holding position
  ST_PLAN_INIT,	  //!< init state, initialize internal states
  ST_PLAN_BUSY,	  //!< busy state, running state machine
  ST_PLAN_HALT,	  //!< halt state, return to the position reference in idle state
  ST_PLAN_WAIT	  //!< wait state, holding the position reference in idle state
} ST_PlanStatus_e;

//! \brief Enumeration for the Plan Condition options
//!
typedef enum {
  ST_COND_NC=0,	//!< Transition/Action with no conditions satisfied
  ST_COND_FC,	//!< Transition/Action with first condition satisfied
  ST_COND_AND,	//!< Transition/Action with both conditions satisfied
  ST_COND_OR	//!< Transition/Action with either condition satisfied
} ST_PlanCond_e;

//! \brief Enumeration for the Plan Compare options
//!
typedef enum {
  ST_COMP_NA=0, //!< No comparison
  ST_COMP_EQ,   //!< Compares, VarIdx == Value1 OR VarIdx1 == VarIdx2
  ST_COMP_NEQ,  //!< Compares, VarIdx != Value1 OR VarIdx1 != VarIdx2
  ST_COMP_GT,   //!< Compares, VarIdx > Value1  OR VarIdx1 > VarIdx2
  ST_COMP_EGT,  //!< Compares, VarIdx >= Value1 OR VarIdx1 >= VarIdx2
  ST_COMP_LW,   //!< Compares, VarIdx < Value1  OR VarIdx1 < VarIdx2
  ST_COMP_ELW,  //!< Compares, VarIdx <= Value1 OR VarIdx1 <= VarIdx2
  ST_COMP_In,   //!< Compares, Value1 <= VarIdx <= Value2
  ST_COMP_EIn,  //!< Compares, Value1 < VarIdx <= Value2
  ST_COMP_InE,  //!< Compares, Value1 <= VarIdx < Value2
  ST_COMP_EInE, //!< Compares, Value1 < VarIdx < Value2
  ST_COMP_Out,  //!< Compares, Value1 >= VarIdx >= Value2
  ST_COMP_EOut, //!< Compares, Value1 > VarIdx >= Value2
  ST_COMP_OutE, //!< Compares, Value1 >= VarIdx > Value2
  ST_COMP_EOutE //!< Compares, Value1 > VarIdx > Value2
} ST_PlanComp_e;

//! \brief Enumeration for the Plan Action Options
//!
typedef enum {
  ST_ACT_EQ=0, //!< Action will set the variable equal to a value
  ST_ACT_ADD   //!< Action will add a value to the variable
} ST_PlanActOptn_e;

//! \brief Enumeration for the Plan Action trigger point types
//!
typedef enum {
  ST_ACT_ENTR=0, //!< Action will be considered when entering the state
  ST_ACT_EXIT    //!< Action will be considered when exiting the state
} ST_PlanActTrgr_e;

//! \brief Enumeration for the Plan FSM states
//!
typedef enum {
  ST_FSM_STATE_STAY=0,	//!< Plan is staying in a state until the timer is up
  ST_FSM_STATE_COND,	//!< Plan is waiting for a transition condition to be true
  ST_FSM_STATE_TRAN		//!< Plan is in transition with a motion profile
} ST_PlanFsmState_e;

//! \brief Enumeration for the Plan variable types
//!
typedef enum {
  ST_VAR_INOUT=0,	//!< Variable type input/output
  ST_VAR_IN,		//!< Variable type input
  ST_VAR_OUT		//!< Variable type output
} ST_PlanVar_e;
#endif //__ST_PLAN_ENUM__

#ifndef __ST_PLAN_ERROR__
#define __ST_PLAN_ERROR__
//! \brief      Defines the ST_PlanError_t data
//! \details    The ST_PlanError_t object contains error debugging information
//!				for the Position Plan component
typedef struct {
	uint16_t ERR_idx;	//!< Plan component index that caused the error
	uint16_t ERR_code;	//!< Function specific condition that caused the error
} ST_PlanError_t;	// Structure for SpinTAC Plan Error
#endif //__ST_PLAN_ERROR__

//! \brief      Defines the ST_PosPlan_t data
//! \details    The ST_PosPlan_t object contains all configuration parameters
//!				of the Position Plan component
typedef struct {
  /* Input variables */
  // Control bits
  bool ENB;                    //!< Enable bit { false: disabled; true: enabled }
  bool RES;                    //!< Reset bit { false: not reset; true: reset }
  /* Output variables */
  int32_t PosStepInt_mrev;     //!< Position Step Integer part { unit: [MRev] }
  _iq24	PosStepFrac_mrev;      //!< Position Step fraction part { unit: [MRev], value range: (-1.0, 1.0)}
  _iq24	VelLim;                //!< Velocity limit { unit: [pu/s], value range: (0.0, 1.0] }
  _iq24	AccLim;                //!< Acceleration limit { unit: [pu/s^2], value range: [0.001, 120.0] }
  _iq24	DecLim;                //!< Deceleration limit { unit: [pu/s^2], value range: [0.001, 120.0] }
  _iq20	JrkLim;                //!< Jerk Limit { unit: [pu/s^3], value range: [0.0005, 2000.0] }
  // Information variables
  ST_PlanStatus_e STATUS;      //!< Plan status { ST_PLAN_IDLE, ST_PLAN_INIT, ST_PLAN_BUSY, ST_PLAN_HALT, ST_PLAN_WAIT}
  uint16_t CurState;           //!< Current state of the state machine
  uint16_t CurTran;            //!< Current transition of the state machine
  ST_PlanFsmState_e FsmState;  //!< State of the state machine
  int32_t Timer_tick;          //!< State Timer { unit: [tick] }
  uint16_t ERR_ID;             //!< Error ID { 0: no error; others: see error code }
  ST_PlanError_t CfgError;     //!< Error decoding structure
  /* Internal variables */
  uint32_t s0[31];
} ST_PosPlan_t;

typedef struct _ST_POSPLAN_Handle_ *ST_POSPLAN_Handle; // SpinTAC Position Plan Handle

//! \brief      Gets the Position Step (posStepInt_mrev, posStepFrac_mrev) for SpinTAC Position Plan
//! \param[in]  handle            The handle for the SpinTAC Position Plan Object
//! \param[out] *posStepInt_mrev  Position Step integer part value { unit: [MRev], value range: 32-bit integer }
//! \param[out] *posStepFrac_mrev Position Step fraction part value { unit: [MRev], value range: (-1.0, 1.0) }
static inline void STPOSPLAN_getPositionStep_mrev(ST_POSPLAN_Handle handle, int32_t *posStepInt_mrev, _iq24 *posStepFrac_mrev) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;
	*posStepInt_mrev = obj->PosStepInt_mrev;
	*posStepFrac_mrev = obj->PosStepFrac_mrev;
	return;
} // end of STPOSPLAN_getPositionStepInt_mrev function

//! \brief      Gets the Velocity Limit (VelLim) for SpinTAC Position Plan
//! \param[in]  handle       The handle for the SpinTAC Position Plan Object
//! \return     _iq24 VelLim Velocity limit { unit: [pu/s], value range: [-1.0, 1.0] }
static inline _iq24 STPOSPLAN_getVelocityLimit(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->VelLim);
} // end of STPOSPLAN_getVelocityLimit function

//! \brief      Gets the Acceleration Limit (AccLim) for SpinTAC Position Plan
//! \param[in]  handle       The handle for the SpinTAC Position Plan Object
//! \return     _iq24 AccLim Acceleration limit { unit: [pu/s^2], value range: (0.0, 120.0] }
static inline _iq24 STPOSPLAN_getAccelerationLimit(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->AccLim);
} // end of STPOSPLAN_getAccelerationLimit function

//! \brief      Gets the Deceleration Limit (DecLim) for SpinTAC Position Plan
//! \param[in]  handle       The handle for the SpinTAC Position Plan Object
//! \return     _iq24 DecLim Deceleration limit { unit: [pu/s^2], value range: (0.0, 120.0] }
static inline _iq24 STPOSPLAN_getDecelerationLimit(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->DecLim);
} // end of STPOSPLAN_getDecelerationLimit function

//! \brief      Gets the Jerk Limit (JrkLim) for SpinTAC Position Plan
//! \param[in]  handle       The handle for the SpinTAC Position Plan Object
//! \return     _iq20 JrkLim Jerk Limit { unit: [pu/s^3], value range: (0.0, 2000.0] }
static inline _iq20 STPOSPLAN_getJerkLimit(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->JrkLim);
} // end of STPOSPLAN_getJerkLimit function

//! \brief      Sets the Enable signal (ENB) for SpinTAC Position Plan
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  enb    Enable bit { false: disable; true: enable }
static inline void STPOSPLAN_setEnable(ST_POSPLAN_Handle handle, bool enb) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	obj->ENB = enb;

	return;
} // end of STPOSPLAN_setEnable function

//! \brief      Gets the Enable signal (ENB) for SpinTAC Position Plan
//! \param[in]  handle   The handle for the SpinTAC Position Plan Object
//! \return     bool ENB Enable bit { false: disable; true: enable }
static inline bool STPOSPLAN_getEnable(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return (obj->ENB);
} // end of STPOSPLAN_getEnable function

//! \brief      Sets the Reset signal (RES) for SpinTAC Position Plan
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  res    Reset bit { false: reset; true: not reset }
static inline void STPOSPLAN_setReset(ST_POSPLAN_Handle handle, bool res) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	obj->RES = res;

	return;
} // end of STPOSPLAN_setReset function

//! \brief      Gets the Reset signal (RES) for SpinTAC Position Plan
//! \param[in]  handle   The handle for the SpinTAC Position Plan Object
//! \return     bool RES Reset bit { false: reset; true: not reset }
static inline bool STPOSPLAN_getReset(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return (obj->RES);
} // end of STPOSPLAN_getReset function

//! \brief      Gets the Status value (STATUS) for SpinTAC Position Plan
//! \param[in]  handle                 The handle for the SpinTAC Position Plan Object
//! \return     ST_PlanStatus_e STATUS Status { ST_VEL_ID_IDLE, ST_VEL_ID_INIT, ST_VEL_ID_BUSY }
static inline ST_PlanStatus_e STPOSPLAN_getStatus(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return (obj->STATUS);
} // end of STPOSPLAN_getStatus function


//! \brief      Gets the Current State (CurState) for SpinTAC Position Plan
//! \param[in]  handle            The handle for the SpinTAC Position Plan Object
//! \return     uint16_t CurState Current state of the state machine
static inline uint16_t STPOSPLAN_getCurrentState(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->CurState);
} // end of STPOSPLAN_getCurrentState function

//! \brief      Gets the Current Transition (CurTran) for SpinTAC Position Plan
//! \param[in]  handle           The handle for the SpinTAC Position Plan Object
//! \return     uint16_t CurTran Current transition of the state machine
static inline uint16_t STPOSPLAN_getCurrentTransition(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->CurTran);
} // end of STPOSPLAN_getCurrentTransition function

//! \brief      Gets the State Machine State (FsmState) for SpinTAC Position Plan
//! \param[in]  handle                     The handle for the SpinTAC Position Plan Object
//! \return     ST_PlanFsmState_e FsmState Current operation of the state machine
static inline ST_PlanFsmState_e STPOSPLAN_getFsmState(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->FsmState);
} // end of STPOSPLAN_getFsmState function

//! \brief      Gets the Current Time Value (Timer_tick) for SpinTAC Position Plan
//! \param[in]  handle        The handle for the SpinTAC Position Plan Object
//! \return     int32_t Timer State Timer { unit: [tick] }
static inline int32_t STPOSPLAN_getCurrentTimerValue_tick(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return(obj->Timer_tick);
} // end of STPOSPLAN_getCurrentTimerValue_tick function

//! \brief      Gets the Configuration Error for SpinTAC Position Plan
//! \param[in]  handle          The handle for the SpinTAC Position Plan Object
//! \param[out] *ERR_idx        Component index where error occurred { 0: no index; others: see error code }
//! \param[out] *ERR_code       Specific configuration error { 0: no additional information; others: see error code }
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STPOSPLAN_getCfgError(ST_POSPLAN_Handle handle, uint16_t *ERR_idx, uint16_t *ERR_code) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	*ERR_idx = obj->CfgError.ERR_idx;
	*ERR_code = obj->CfgError.ERR_code;

	return (obj->ERR_ID);
} // end of STPOSPLAN_getCfgError function

//! \brief      Gets the Error value (ERR_ID) for SpinTAC Position Plan
//! \param[in]  handle          The handle for the SpinTAC Position Plan Object
//! \return     uint16_t ERR_ID Error ID { 0: no error; others: see error code }
static inline uint16_t STPOSPLAN_getErrorID(ST_POSPLAN_Handle handle) {
	ST_PosPlan_t *obj = (ST_PosPlan_t *)handle;

	return (obj->ERR_ID);
} // end of STPOSPLAN_getErrorID function

//! \brief      Returns the number of configured States in SpinTAC Position Plan
//! \param[in]  handle    The handle for the SpinTAC Position Plan Object
//! \param[out] *StateNum The number of configured States in SpinTAC Position Plan
void STPOSPLAN_getCfgStateNum(ST_POSPLAN_Handle handle, uint16_t *StateNum);

//! \brief      Returns the number of configured Variables in SpinTAC Position Plan
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[out] *VarNum The number of configured Variables in SpinTAC Position Plan
void STPOSPLAN_getCfgVarNum(ST_POSPLAN_Handle handle, uint16_t *VarNum);

//! \brief      Returns the number of configured Conditions in SpinTAC Position Plan
//! \param[in]  handle   The handle for the SpinTAC Position Plan Object
//! \param[out] *CondNum The number of configured Conditions in SpinTAC Position Plan
void STPOSPLAN_getCfgCondNum(ST_POSPLAN_Handle handle, uint16_t *CondNum);

//! \brief      Returns the number of configured Transitions in SpinTAC Position Plan
//! \param[in]  handle   The handle for the SpinTAC Position Plan Object
//! \param[out] *TranNum The number of configured Transitions in SpinTAC Position Plan
void STPOSPLAN_getCfgTranNum(ST_POSPLAN_Handle handle, uint16_t *TranNum);

//! \brief      Returns the number of configured Actions in SpinTAC Position Plan
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[out] *ActNum The number of configured Actions in SpinTAC Position Plan
void STPOSPLAN_getCfgActNum(ST_POSPLAN_Handle handle, uint16_t *ActNum);

//! \brief      Adds a Condition to the SpinTAC Position Plan configuration
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx Index of the Plan Variable that will be compared
//! \param[in]  Comp   Type of comparison that will be done
//! \param[in]  Value1 First value to use in the comparison
//! \param[in]  Value2 Second value to use in the comparison
void STPOSPLAN_addCfgCond(ST_POSPLAN_Handle handle, uint16_t VarIdx, ST_PlanComp_e Comp, _iq24 Value1, _iq24 Value2);

//! \brief      Deletes a Condition from the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  CondIdx Index of the Plan Condition to delete
void STPOSPLAN_delCfgCond(ST_POSPLAN_Handle handle, uint16_t CondIdx);

//! \brief      Modifies a Condition in the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  CondIdx Index of the Plan Condition to modify
//! \param[in]  VarIdx  Index of the Plan Variable that will be compared
//! \param[in]  Comp    Type of comparison that will be done
//! \param[in]  Value1  First value to use in the comparison
//! \param[in]  Value2  Second value to use in the comparison
void STPOSPLAN_setCfgCond(ST_POSPLAN_Handle handle, uint16_t CondIdx, uint16_t VarIdx, ST_PlanComp_e Comp, _iq24 Value1, _iq24 Value2);

//! \brief      Returns a Condition from the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  CondIdx Index of the Plan Condition to return
//! \param[out] *VarIdx Index of the Plan Variable that will be compared
//! \param[out] *Comp   Type of comparison that will be done
//! \param[out] *Value1 First value to use in the comparison
//! \param[out] *Value2 Second value to use in the comparison
void STPOSPLAN_getCfgCond(ST_POSPLAN_Handle handle, uint16_t CondIdx, uint16_t *VarIdx, ST_PlanComp_e *Comp, _iq24 *Value1, _iq24 *Value2);

//! \brief      Adds a Variable Condition to the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx1 Index of the first Plan Variable that will be compared
//! \param[in]  VarIdx2 Index of the second Plan Variable that will be compared
//! \param[in]  Comp    Type of comparison that will be done
void STPOSPLAN_addCfgVarCond(ST_POSPLAN_Handle handle, uint16_t VarIdx1, uint16_t VarIdx2, ST_PlanComp_e Comp);

//! \brief      Deletes a Variable Condition from the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  CondIdx Index of the Plan Condition to delete
void STPOSPLAN_delCfgVarCond(ST_POSPLAN_Handle handle, uint16_t CondIdx);

//! \brief      Modifies a Variable Condition to the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  CondIdx Index of the Plan Condition to modify
//! \param[in]  VarIdx1 Index of the first Plan Variable that will be compared
//! \param[in]  VarIdx2 Index of the second Plan Variable that will be compared
//! \param[in]  Comp    Type of comparison that will be done
void STPOSPLAN_setCfgVarCond(ST_POSPLAN_Handle handle, uint16_t CondIdx, uint16_t VarIdx1, uint16_t VarIdx2, ST_PlanComp_e Comp);

//! \brief      Returns a Variable Condition from the SpinTAC Position Plan configuration
//! \param[in]  handle   The handle for the SpinTAC Position Plan Object
//! \param[in]  CondIdx  Index of the Plan Condition to return
//! \param[out] *VarIdx1 Index of the first Plan Variable that will be compared
//! \param[out] *VarIdx2 Index of the second Plan Variable that will be compared
//! \param[out] *Comp    Type of comparison that will be done
void STPOSPLAN_getCfgVarCond(ST_POSPLAN_Handle handle, uint16_t CondIdx, uint16_t *VarIdx1, uint16_t *VarIdx2, ST_PlanComp_e *Comp);

//! \brief      Adds a Transition to the SpinTAC Position Plan configuration
//! \param[in]  handle    The handle for the SpinTAC Position Plan Object
//! \param[in]  FromState Index of the from State
//! \param[in]  ToState   Index of the to State
//! \param[in]  AndOr     Condition option for the Transition
//! \param[in]  CondIdx1  Index of the first Condition to consider for the Transition
//! \param[in]  CondIdx2  Index of the second Condition to consider for the Transition
//! \param[in]  VelLim    Velocity limit for the Transition { unit: [pu/s], value range: (0.0, 1.0] }
//! \param[in]  AccLim    Acceleration limit for the Transition { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[in]  DecLim    Deceleration limit for the Transition { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[in]  JrkLim    Jerk limit for the Transition { unit: [pu/s^3], value range: [0.0005, 2000.0] }
void STPOSPLAN_addCfgTran(ST_POSPLAN_Handle handle, uint16_t FromState, uint16_t ToState, ST_PlanCond_e AndOr, uint16_t CondIdx1, uint16_t CondIdx2, _iq24 VelLim, _iq24 AccLim, _iq24 DecLim, _iq20 JrkLim);

//! \brief      Deletes a Transition from the SpinTAC Position Plan configuration
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in]  TranIdx Index of the Transition to delete
void STPOSPLAN_delCfgTran(ST_POSPLAN_Handle handle, uint16_t TranIdx);

//! \brief      Modifies a Transition in the SpinTAC Position Plan configuration
//! \param[in]  handle    The handle for the SpinTAC Position Plan Object
//! \param[in]  TranIdx   Index of the Transition to modify
//! \param[in]  FromState Index of the from State
//! \param[in]  ToState   Index of the to State
//! \param[in]  AndOr     Condition option for the Transition
//! \param[in]  CondIdx1  Index of the first Condition to consider for the Transition
//! \param[in]  CondIdx2  Index of the second Condition to consider for the Transition
//! \param[in]  VelLim    Velocity limit for the Transition { unit: [pu/s], value range: [-1.0, 1.0] }
//! \param[in]  AccLim    Acceleration limit for the Transition { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[in]  DecLim    Deceleration limit for the Transition { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[in]  JrkLim    Jerk limit for the Transition { unit: [pu/s^3], value range: [0.0005, 2000.0] }
void STPOSPLAN_setCfgTran(ST_POSPLAN_Handle handle, uint16_t TranIdx, uint16_t FromState, uint16_t ToState, ST_PlanCond_e AndOr, uint16_t CondIdx1, uint16_t CondIdx2, _iq24 VelLim, _iq24 AccLim, _iq24 DecLim, _iq20 JrkLim);

//! \brief      Get a Transition from the SpinTAC Position Plan configuration
//! \param[in]  handle     The handle for the SpinTAC Position Plan Object
//! \param[in]  TranIdx    Index of the Transition to return
//! \param[out] *FromState Index of the from State
//! \param[out] *ToState   Index of the to State
//! \param[out] *AndOr     Condition option for the Transition
//! \param[out] *CondIdx1  Index of the first Condition to consider for the Transition
//! \param[out] *CondIdx2  Index of the second Condition to consider for the Transition
//! \param[out] *VelLim    Velocity limit for the Transition { unit: [pu/s], value range: (0.0, 1.0] }
//! \param[out] *AccLim    Acceleration limit for the Transition { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[out] *DecLim    Deceleration limit for the Transition { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[out] *JrkLim    Jerk limit for the Transition { unit: [pu/s^3], value range: [0.0005, 2000.0] }
void STPOSPLAN_getCfgTran(ST_POSPLAN_Handle handle, uint16_t TranIdx, uint16_t *FromState, uint16_t *ToState, ST_PlanCond_e *AndOr, uint16_t *CondIdx1, uint16_t *CondIdx2, _iq24 *VelLim, _iq24 *AccLim, _iq24 *DecLim, _iq20 *JrkLim);

//! \brief      Adds an action to the SpinTAC Position Plan configuration
//! \param[in]  handle    The handle for the SpinTAC Position Plan Object
//! \param[in]  State     Index of the State the Action happens in
//! \param[in]  AndOr     Condition option for the Action
//! \param[in]  CondIdx1  Index of the first Condition to consider for the Action
//! \param[in]  CondIdx2  Index of the second Condition to consider for the Action
//! \param[in]  VarIdx    Index of the Varaible to be acted upon
//! \param[in]  Opt       Type of Action to perform on the Varaible
//! \param[in]  Value     Value to use in the Action
//! \param[in]  EnterExit State event that will trigger the Action
void STPOSPLAN_addCfgAct(ST_POSPLAN_Handle handle, uint16_t State, ST_PlanCond_e AndOr, uint16_t CondIdx1, uint16_t CondIdx2, uint16_t VarIdx, ST_PlanActOptn_e Opt, _iq24 Value, ST_PlanActTrgr_e EnterExit);

//! \brief      Deletes an action from the SpinTAC Position Plan configuration
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  ActIdx Index of the Action to delete
void STPOSPLAN_delCfgAct(ST_POSPLAN_Handle handle, uint16_t ActIdx);

//! \brief      Modifies an action in the SpinTAC Position Plan configuration
//! \param[in]  handle    The handle for the SpinTAC Position Plan Object
//! \param[in]  ActIdx    Index of the Action to modify
//! \param[in]  State     Index of the State the Action happens in
//! \param[in]  AndOr     Condition option for the Action
//! \param[in]  CondIdx1  Index of the first Condition to consider for the Action
//! \param[in]  CondIdx2  Index of the second Condition to consider for the Action
//! \param[in]  VarIdx    Index of the Varaible to be acted upon
//! \param[in]  Opt       Type of Action to perform on the Varaible
//! \param[in]  Value     Value to use in the Action
//! \param[in]  EnterExit State event that will trigger the Action
void STPOSPLAN_setCfgAct(ST_POSPLAN_Handle handle, uint16_t ActIdx, uint16_t State, ST_PlanCond_e AndOr, uint16_t CondIdx1, uint16_t CondIdx2, uint16_t VarIdx, ST_PlanActOptn_e Opt, _iq24 Value, ST_PlanActTrgr_e EnterExit);

//! \brief      Returns an action from the SpinTAC Position Plan configuration
//! \param[in]  handle     The handle for the SpinTAC Position Plan Object
//! \param[in]  ActIdx     Index of the Action to return
//! \param[out] *State     Index of the State the Action happens in
//! \param[out] *AndOr     Condition option for the Action
//! \param[out] *CondIdx1  Index of the first Condition to consider for the Action
//! \param[out] *CondIdx2  Index of the second Condition to consider for the Action
//! \param[out] *VarIdx    Index of the Varaible to be acted upon
//! \param[out] *Opt       Type of Action to perform on the Varaible
//! \param[out] *Value     Value to use in the Action
//! \param[out] *EnterExit State event that will trigger the Action
void STPOSPLAN_getCfgAct(ST_POSPLAN_Handle handle, uint16_t ActIdx, uint16_t *State, ST_PlanCond_e *AndOr, uint16_t *CondIdx1, uint16_t *CondIdx2, uint16_t *VarIdx, ST_PlanActOptn_e *Opt, _iq24 *Value, ST_PlanActTrgr_e *EnterExit);

//! \brief      Adds a variable to the SpinTAC Position Plan configuration
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  Type   Type of Variable
//! \param[in]  Value  Initial value of the Variable
void STPOSPLAN_addCfgVar(ST_POSPLAN_Handle handle, ST_PlanVar_e Type, _iq24 Value);

//! \brief      Deletes a Variable from the SpinTAC Position Plan configuration
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx Index of the Variable to delete
void STPOSPLAN_delCfgVar(ST_POSPLAN_Handle handle, uint16_t VarIdx);

//! \brief      Modifies a variable in the SpinTAC Position Plan configuration
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx Index of the Variable to modify
//! \param[in]  Type   Type of Variable
//! \param[in]  Value  Initial value of the Variable
void STPOSPLAN_setCfgVar(ST_POSPLAN_Handle handle, uint16_t VarIdx, ST_PlanVar_e Type, _iq24 Value);

//! \brief      Returns a variable from the SpinTAC Position Plan configuration
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx Index of the Variable to return
//! \param[out] *Type  Type of Variable
//! \param[out] *Value Initial value of the Variable
void STPOSPLAN_getCfgVar(ST_POSPLAN_Handle handle, uint16_t VarIdx, ST_PlanVar_e *Type, _iq24 *Value);

//! \brief      Sets the value of a SpinTAC Position Plan variable 
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx Index of the Variable to set
//! \param[in]  Value  Value to set to the Variable
void STPOSPLAN_setVar(ST_POSPLAN_Handle handle, uint16_t VarIdx, _iq24 Value);

//! \brief      Gets the value of a SpinTAC Position Plan variable
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  VarIdx Index of the Variable to get
//! \param[out] *Value Value to get from the Variable
void STPOSPLAN_getVar(ST_POSPLAN_Handle handle, uint16_t VarIdx, _iq24 *Value);

//! \brief      Sets a flag in SpinTAC Position Plan to indicate if the unit profile is complete
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
//! \param[in]  ProDON Indicates if the unit profile is completed
void STPOSPLAN_setUnitProfDone(ST_POSPLAN_Handle handle, bool ProDON);

//! \brief      Configures SpinTAC Position Plan
//! \param[in]  handle  The handle for the SpinTAC Position Plan Object
//! \param[in] 	T_sec   Sample Time { unit: [sec], value range: (0, 0.01] }
//! \param[in]  LoopENB Sets if SpinTAC Position Plan should continuously run after it reaches the end { false: Do not continuously run Plan; true: Continuously run Plan }
void STPOSPLAN_setCfg(ST_POSPLAN_Handle handle, _iq24 T_sec, bool LoopENB);

//! \brief      Gets the SpinTAC Position Plan configuration
//! \param[in]  handle   handle The handle for the SpinTAC Position Plan Object
//! \param[out] *T_sec   Sample Time { unit: [sec], value range: (0, 0.01] }
//! \param[out] *LoopENB Sets if SpinTAC Position Plan should continuously run after it reaches the end { false: Do not continuously run Plan; true: Continuously run Plan }
void STPOSPLAN_getCfg(ST_POSPLAN_Handle handle, _iq24 *T_sec, bool *LoopENB);

//! \brief      Configures the SpinTAC Position Plan Halt state
//! \param[in]  handle           The handle for the SpinTAC Position Plan Object
//! \param[in]  PosStepInt_mrev  Position step integer part for Halt State { unit: [MRev], value range: 32 bit integer [-2, 2] }
//! \param[in]  PosStepFrac_mrev Position step fraction part for Halt State { unit: [MRev], value range: (-1.0, 1.0) }
//! \param[in]  VelLim           Velocity limit during Halt state { unit: [pu/s], value range: (0, 1] }
//! \param[in]  AccLim           Acceleration limit for Halt state { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[in]  JrkLim           Jerk limit for Halt state { unit: [pu/s^3], value range: [0.0005, 2000.0] }
//! \param[in]  Timer_tick       Minimum amount of time to stay in Halt state { unit: [tick] }
void STPOSPLAN_setCfgHaltState(ST_POSPLAN_Handle handle, int32_t PosStepInt_mrev, _iq24 PosStepFrac_mrev, _iq24 VelLim, _iq24 AccLim, _iq20 JrkLim, int32_t Timer_tick);

//! \brief      Gets the SpinTAC Position Plan Halt state
//! \param[in]  handle            The handle for the SpinTAC Position Plan Object
//! \param[out] *PosStepInt_mrev  Position step integer part for Halt State { unit: [MRev] }
//! \param[out] *PosStepFrac_mrev Position step fraction part for Halt State { unit: [MRev], value range: (-1.0, 1.0) }
//! \param[out] *VelLim           Velocity limit during Halt state { unit: [pu/s], value range: (0, 1] }
//! \param[out] *AccLim           Acceleration limit for Halt state { unit: [pu/s^2], value range: [0.001, 120.0] }
//! \param[our] *JrkLim           Jerk limit for Halt state { unit: [pu/s^3], value range: [0.0005, 2000.0] }
//! \param[out] *Timer_tick       Minimum amount of time to stay in Halt state { unit: [tick] }
void STPOSPLAN_getCfgHaltState(ST_POSPLAN_Handle handle, int32_t *PosStepInt_mrev, _iq24 *PosStepFrac_mrev, _iq24 *VelLim, _iq24 *AccLim, _iq20 *JrkLim, int32_t *Timer_tick);

//! \brief      Adds a State to the SpinTAC Position Plan configuration
//! \param[in]  handle           The handle for the SpinTAC Position Plan Object
//! \param[in]  PosStepInt_mrev  Position step integer part in State { unit: [MRev], value range: 32 bit integer (-2^31, 2^31) }
//! \param[in]  PosStepFrac_mrev Position step fraction part in State { unit: [MRev], value range: (-1.0, 1.0) }
//! \param[in]  Timer_tick       Minimum time to stay in State { unit: [tick] }
void STPOSPLAN_addCfgState(ST_POSPLAN_Handle handle, int32_t PosStepInt_mrev, _iq24 PosStep_Fracmrev, int32_t Timer_tick);

//! \brief      Deletes a state from the SpinTAC Position Plan configuration
//! \param[in]  handle   The handle for the SpinTAC Position Plan Object
//! \param[in]  StateIdx Index of the State to delete
void STPOSPLAN_delCfgState(ST_POSPLAN_Handle handle, uint16_t StateIdx);

//! \brief      Modifies a state in the SpinTAC Position Plan configuration
//! \param[in]  handle           The handle for the SpinTAC Position Plan Object
//! \param[in]  StateIdx         Index of the State to modify
//! \param[in]  PosStepInt_mrev  Position step integer part in State { unit: [MRev], value range: 32 bit integer (-2^31, 2^31) }
//! \param[in]  PosStepFrac_mrev Position step fraction part in State { unit: [MRev], value range: (-1.0, 1.0) }
//! \param[in]  Timer_tick       Minimum time to stay in State { unit: [tick] }
void STPOSPLAN_setCfgState(ST_POSPLAN_Handle handle, uint16_t StateIdx, int32_t PosStepInt_mrev, _iq24 PosStepFrac_mrev, int32_t Timer_tick);

//! \brief      Returns a state from the SpinTAC Position Plan configuration
//! \param[in]  handle            The handle for the SpinTAC Position Plan Object
//! \param[in]  StateIdx          Index of the State to modify
//! \param[out] *PosStepInt_mrev  Position step integer part in State { unit: [MRev] }
//! \param[out] *PosStepFrac_mrev Position step fraction part in State { unit: [MRev] }
//! \param[out] *Timer_tick       Minimum time to stay in State { unit: [tick] }
void STPOSPLAN_getCfgState(ST_POSPLAN_Handle handle, uint16_t StateIdx, int32_t *PosStepInt_mrev, _iq24 *PosStepFrac_mrev, int32_t *Timer_tick);

//! \brief      Resets the SpinAC Position Plan component
//! \param[in]  handle The handle for the SpinTAC Position Plan Object
void STPOSPLAN_reset(ST_POSPLAN_Handle handle);

//! \brief      Initializes the SpinTAC Position Plan component
//! \param[in]  *pMemory                 Pointer to the memory for ST_PosPlan_t
//! \param[in]  numBytes                 The number of bytes in the ST_PosPlan_t
//! \return     ST_POSPLAN_Handle handle The handle for the SpinTAC Position Plan Object
ST_POSPLAN_Handle STPOSPLAN_init(void *pMemory, const size_t numBytes);

//! \brief      Runs SpinTAC Position Plan calculation
//! \param[in] 	handle The handle for the Position Plan structure
void STPOSPLAN_run(ST_POSPLAN_Handle handle);

//! \brief      Runs SpinTAC Position Plan Timing. Call this only from an ISR.
//! \param[in] 	handle The handle for the Position Plan structure
void STPOSPLAN_runTick(ST_POSPLAN_Handle handle);

//! \brief      Prepares SpinTAC Position Plan data structure
//! \param[in] 	handle      The handle for the Position Plan structure
//! \param[in]  *cfgArray   Pointer to the SpinTAC Position Plan configuration array
//! \param[in]  numBytes    The number of bytes in the SpinTAC Position Plan configuration array
//! \param[in]  MaxActNum   Number of Actions
//! \param[in]  MaxCondNum  Number of Conditions
//! \param[in]  MaxVarNum   Number of Variables
//! \param[in]  MaxTranNum  Number of Transitions
//! \param[in]  MaxStateNum Number of States
void STPOSPLAN_setCfgArray(ST_POSPLAN_Handle handle, uint32_t *cfgArray, const size_t numBytes, uint16_t MaxActNum, uint16_t MaxCondNum, uint16_t MaxVarNum, uint16_t MaxTranNum, uint16_t MaxStateNum);
//@} // defgroup

#endif /*__SPINTAC_POS_PLAN_H__*/
