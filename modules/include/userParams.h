#ifndef _USERPARAMS_H_
#define _USERPARAMS_H_
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

//! \file   solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src/user.h
//! \brief Contains the public interface for user initialization data for the CTRL, HAL, and EST modules
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "types.h"
#include "motor.h"

#include "est_states.h"
#include "est_Flux_states.h"
#include "est_Ls_states.h"
#include "est_Rs_states.h"

#include "ctrl_states.h"


//!
//!
//! \defgroup USERPARAMS USERPARAMS
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines




// **************************************************************************
// the typedefs

//! \brief Enumeration for the user error codes
//!
typedef enum
{
  USER_ErrorCode_NoError=0,                           //!< no error error code
  USER_ErrorCode_iqFullScaleCurrent_A_High=1,         //!< iqFullScaleCurrent_A too high error code
  USER_ErrorCode_iqFullScaleCurrent_A_Low=2,          //!< iqFullScaleCurrent_A too low error code
  USER_ErrorCode_iqFullScaleVoltage_V_High=3,         //!< iqFullScaleVoltage_V too high error code
  USER_ErrorCode_iqFullScaleVoltage_V_Low=4,          //!< iqFullScaleVoltage_V too low error code
  USER_ErrorCode_iqFullScaleFreq_Hz_High=5,           //!< iqFullScaleFreq_Hz too high error code
  USER_ErrorCode_iqFullScaleFreq_Hz_Low=6,            //!< iqFullScaleFreq_Hz too low error code
  USER_ErrorCode_numPwmTicksPerIsrTick_High=7,        //!< numPwmTicksPerIsrTick too high error code
  USER_ErrorCode_numPwmTicksPerIsrTick_Low=8,         //!< numPwmTicksPerIsrTick too low error code
  USER_ErrorCode_numIsrTicksPerCtrlTick_High=9,       //!< numIsrTicksPerCtrlTick too high error code
  USER_ErrorCode_numIsrTicksPerCtrlTick_Low=10,       //!< numIsrTicksPerCtrlTick too low error code
  USER_ErrorCode_numCtrlTicksPerCurrentTick_High=11,  //!< numCtrlTicksPerCurrentTick too high error code
  USER_ErrorCode_numCtrlTicksPerCurrentTick_Low=12,   //!< numCtrlTicksPerCurrentTick too low error code
  USER_ErrorCode_numCtrlTicksPerEstTick_High=13,      //!< numCtrlTicksPerEstTick too high error code
  USER_ErrorCode_numCtrlTicksPerEstTick_Low=14,       //!< numCtrlTicksPerEstTick too low error code
  USER_ErrorCode_numCtrlTicksPerSpeedTick_High=15,    //!< numCtrlTicksPerSpeedTick too high error code
  USER_ErrorCode_numCtrlTicksPerSpeedTick_Low=16,     //!< numCtrlTicksPerSpeedTick too low error code
  USER_ErrorCode_numCtrlTicksPerTrajTick_High=17,     //!< numCtrlTicksPerTrajTick too high error code
  USER_ErrorCode_numCtrlTicksPerTrajTick_Low=18,      //!< numCtrlTicksPerTrajTick too low error code
  USER_ErrorCode_numCurrentSensors_High=19,           //!< numCurrentSensors too high error code
  USER_ErrorCode_numCurrentSensors_Low=20,            //!< numCurrentSensors too low error code
  USER_ErrorCode_numVoltageSensors_High=21,           //!< numVoltageSensors too high error code
  USER_ErrorCode_numVoltageSensors_Low=22,            //!< numVoltageSensors too low error code
  USER_ErrorCode_offsetPole_rps_High=23,              //!< offsetPole_rps too high error code
  USER_ErrorCode_offsetPole_rps_Low=24,               //!< offsetPole_rps too low error code
  USER_ErrorCode_fluxPole_rps_High=25,                //!< fluxPole_rps too high error code
  USER_ErrorCode_fluxPole_rps_Low=26,                 //!< fluxPole_rps too low error code
  USER_ErrorCode_zeroSpeedLimit_High=27,              //!< zeroSpeedLimit too high error code
  USER_ErrorCode_zeroSpeedLimit_Low=28,               //!< zeroSpeedLimit too low error code
  USER_ErrorCode_forceAngleFreq_Hz_High=29,           //!< forceAngleFreq_Hz too high error code
  USER_ErrorCode_forceAngleFreq_Hz_Low=30,            //!< forceAngleFreq_Hz too low error code
  USER_ErrorCode_maxAccel_Hzps_High=31,               //!< maxAccel_Hzps too high error code
  USER_ErrorCode_maxAccel_Hzps_Low=32,                //!< maxAccel_Hzps too low error code
  USER_ErrorCode_maxAccel_est_Hzps_High=33,           //!< maxAccel_est_Hzps too high error code
  USER_ErrorCode_maxAccel_est_Hzps_Low=34,            //!< maxAccel_est_Hzps too low error code
  USER_ErrorCode_directionPole_rps_High=35,           //!< directionPole_rps too high error code
  USER_ErrorCode_directionPole_rps_Low=36,            //!< directionPole_rps too low error code
  USER_ErrorCode_speedPole_rps_High=37,               //!< speedPole_rps too high error code
  USER_ErrorCode_speedPole_rps_Low=38,                //!< speedPole_rps too low error code
  USER_ErrorCode_dcBusPole_rps_High=39,               //!< dcBusPole_rps too high error code
  USER_ErrorCode_dcBusPole_rps_Low=40,                //!< dcBusPole_rps too low error code
  USER_ErrorCode_fluxFraction_High=41,                //!< fluxFraction too high error code
  USER_ErrorCode_fluxFraction_Low=42,                 //!< fluxFraction too low error code
  USER_ErrorCode_indEst_speedMaxFraction_High=43,     //!< indEst_speedMaxFraction too high error code
  USER_ErrorCode_indEst_speedMaxFraction_Low=44,      //!< indEst_speedMaxFraction too low error code
  USER_ErrorCode_powerWarpGain_High=45,               //!< powerWarpGain too high error code
  USER_ErrorCode_powerWarpGain_Low=46,                //!< powerWarpGain too low error code
  USER_ErrorCode_systemFreq_MHz_High=47,              //!< systemFreq_MHz too high error code
  USER_ErrorCode_systemFreq_MHz_Low=48,               //!< systemFreq_MHz too low error code
  USER_ErrorCode_pwmFreq_kHz_High=49,                 //!< pwmFreq_kHz too high error code
  USER_ErrorCode_pwmFreq_kHz_Low=50,                  //!< pwmFreq_kHz too low error code
  USER_ErrorCode_voltage_sf_High=51,                  //!< voltage_sf too high error code
  USER_ErrorCode_voltage_sf_Low=52,                   //!< voltage_sf too low error code
  USER_ErrorCode_current_sf_High=53,                  //!< current_sf too high error code
  USER_ErrorCode_current_sf_Low=54,                   //!< current_sf too low error code
  USER_ErrorCode_voltageFilterPole_Hz_High=55,        //!< voltageFilterPole_Hz too high error code
  USER_ErrorCode_voltageFilterPole_Hz_Low=56,         //!< voltageFilterPole_Hz too low error code
  USER_ErrorCode_maxVsMag_pu_High=57,                 //!< maxVsMag_pu too high error code
  USER_ErrorCode_maxVsMag_pu_Low=58,                  //!< maxVsMag_pu too low error code
  USER_ErrorCode_estKappa_High=59,                    //!< estKappa too high error code
  USER_ErrorCode_estKappa_Low=60,                     //!< estKappa too low error code
  USER_ErrorCode_motor_type_Unknown=61,               //!< motor type unknown error code
  USER_ErrorCode_motor_numPolePairs_High=62,          //!< motor_numPolePairs too high error code
  USER_ErrorCode_motor_numPolePairs_Low=63,           //!< motor_numPolePairs too low error code
  USER_ErrorCode_motor_ratedFlux_High=64,             //!< motor_ratedFlux too high error code
  USER_ErrorCode_motor_ratedFlux_Low=65,              //!< motor_ratedFlux too low error code
  USER_ErrorCode_motor_Rr_High=66,                    //!< motor_Rr too high error code
  USER_ErrorCode_motor_Rr_Low=67,                     //!< motor_Rr too low error code
  USER_ErrorCode_motor_Rs_High=68,                    //!< motor_Rs too high error code
  USER_ErrorCode_motor_Rs_Low=69,                     //!< motor_Rs too low error code
  USER_ErrorCode_motor_Ls_d_High=70,                  //!< motor_Ls_d too high error code
  USER_ErrorCode_motor_Ls_d_Low=71,                   //!< motor_Ls_d too low error code
  USER_ErrorCode_motor_Ls_q_High=72,                  //!< motor_Ls_q too high error code
  USER_ErrorCode_motor_Ls_q_Low=73,                   //!< motor_Ls_q too low error code
  USER_ErrorCode_maxCurrent_High=74,                  //!< maxCurrent too high error code
  USER_ErrorCode_maxCurrent_Low=75,                   //!< maxCurrent too low error code
  USER_ErrorCode_maxCurrent_resEst_High=76,           //!< maxCurrent_resEst too high error code
  USER_ErrorCode_maxCurrent_resEst_Low=77,            //!< maxCurrent_resEst too low error code
  USER_ErrorCode_maxCurrent_indEst_High=78,           //!< maxCurrent_indEst too high error code
  USER_ErrorCode_maxCurrent_indEst_Low=79,            //!< maxCurrent_indEst too low error code
  USER_ErrorCode_maxCurrentSlope_High=80,             //!< maxCurrentSlope too high error code
  USER_ErrorCode_maxCurrentSlope_Low=81,              //!< maxCurrentSlope too low error code
  USER_ErrorCode_maxCurrentSlope_powerWarp_High=82,   //!< maxCurrentSlope_powerWarp too high error code
  USER_ErrorCode_maxCurrentSlope_powerWarp_Low=83,    //!< maxCurrentSlope_powerWarp too low error code
  USER_ErrorCode_IdRated_High=84,                     //!< IdRated too high error code
  USER_ErrorCode_IdRated_Low=85,                      //!< IdRated too low error code
  USER_ErrorCode_IdRatedFraction_indEst_High=86,      //!< IdRatedFraction_indEst too high error code
  USER_ErrorCode_IdRatedFraction_indEst_Low=87,       //!< IdRatedFraction_indEst too low error code
  USER_ErrorCode_IdRatedFraction_ratedFlux_High=88,   //!< IdRatedFraction_ratedFlux too high error code
  USER_ErrorCode_IdRatedFraction_ratedFlux_Low=89,    //!< IdRatedFraction_ratedFlux too low error code
  USER_ErrorCode_IdRated_delta_High=90,               //!< IdRated_delta too high error code
  USER_ErrorCode_IdRated_delta_Low=91,                //!< IdRated_delta too low error code
  USER_ErrorCode_fluxEstFreq_Hz_High=92,              //!< fluxEstFreq_Hz too high error code
  USER_ErrorCode_fluxEstFreq_Hz_Low=93,               //!< fluxEstFreq_Hz too low error code
  USER_ErrorCode_ctrlFreq_Hz_High=94,                 //!< ctrlFreq_Hz too high error code
  USER_ErrorCode_ctrlFreq_Hz_Low=95,                  //!< ctrlFreq_Hz too low error code
  USER_ErrorCode_estFreq_Hz_High=96,                  //!< estFreq_Hz too high error code
  USER_ErrorCode_estFreq_Hz_Low=97,                   //!< estFreq_Hz too low error code
  USER_ErrorCode_RoverL_estFreq_Hz_High=98,           //!< RoverL_estFreq_Hz too high error code
  USER_ErrorCode_RoverL_estFreq_Hz_Low=99,            //!< RoverL_estFreq_Hz too low error code
  USER_ErrorCode_trajFreq_Hz_High=100,                //!< trajFreq_Hz too high error code
  USER_ErrorCode_trajFreq_Hz_Low=101,                 //!< trajFreq_Hz too low error code
  USER_ErrorCode_ctrlPeriod_sec_High=102,             //!< ctrlPeriod_sec too high error code
  USER_ErrorCode_ctrlPeriod_sec_Low=103,              //!< ctrlPeriod_sec too low error code
  USER_ErrorCode_maxNegativeIdCurrent_a_High=104,     //!< maxNegativeIdCurrent_a too high error code
  USER_ErrorCode_maxNegativeIdCurrent_a_Low=105,      //!< maxNegativeIdCurrent_a too low error code
  USER_numErrorCodes=106                              //!< the number of user error codes
} USER_ErrorCode_e;


//! \brief Defines a structure for the user parameters
//!
typedef struct _USER_Params_
{
  float_t       iqFullScaleCurrent_A;         //!< Defines the full scale current for the IQ variables, A
  float_t       iqFullScaleVoltage_V;         //!< Defines the full scale voltage for the IQ variable, V
  float_t       iqFullScaleFreq_Hz;           //!< Defines the full scale frequency for IQ variable, Hz

  uint_least16_t  numIsrTicksPerCtrlTick;       //!< Defines the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
  uint_least16_t  numCtrlTicksPerCurrentTick;   //!< Defines the number of controller clock ticks per current controller clock tick
  uint_least16_t  numCtrlTicksPerEstTick;       //!< Defines the number of controller clock ticks per estimator clock tick
  uint_least16_t  numCtrlTicksPerSpeedTick;     //!< Defines the number of controller clock ticks per speed controller clock tick
  uint_least16_t  numCtrlTicksPerTrajTick;      //!< Defines the number of controller clock ticks per trajectory clock tick
  uint_least8_t   numCurrentSensors;            //!< Defines the number of current sensors
  uint_least8_t   numVoltageSensors;            //!< Defines the number of voltage sensors

  float_t       offsetPole_rps;               //!< Defines the pole location for the voltage and current offset estimation, rad/s
  float_t       fluxPole_rps;                 //!< Defines the pole location for the flux estimation, rad/s
  float_t       zeroSpeedLimit;               //!< Defines the low speed limit for the flux integrator, pu
  float_t       forceAngleFreq_Hz;            //!< Defines the force angle frequency, Hz
  float_t       maxAccel_Hzps;                //!< Defines the maximum acceleration for the speed profiles, Hz/s
  float_t       maxAccel_est_Hzps;            //!< Defines the maximum acceleration for the estimation speed profiles, Hz/s
  float_t       directionPole_rps;            //!< Defines the pole location for the direction filter, rad/s
  float_t       speedPole_rps;                //!< Defines the pole location for the speed control filter, rad/s
  float_t       dcBusPole_rps;                //!< Defines the pole location for the DC bus filter, rad/s
  float_t       fluxFraction;                 //!< Defines the flux fraction for Id rated current estimation
  float_t       indEst_speedMaxFraction;      //!< Defines the fraction of SpeedMax to use during inductance estimation
  float_t       powerWarpGain;                //!< Defines the PowerWarp gain for computing Id reference

  uint_least16_t  systemFreq_MHz;               //!< Defines the system clock frequency, MHz

  float_t       pwmPeriod_usec;               //!< Defines the Pulse Width Modulation (PWM) period, usec
  float_t       voltage_sf;                   //!< Defines the voltage scale factor for the system
  float_t       current_sf;                   //!< Defines the current scale factor for the system
  float_t       voltageFilterPole_rps;        //!< Defines the analog voltage filter pole location, rad/s
  float_t       maxVsMag_pu;                  //!< Defines the maximum voltage magnitude, pu
  float_t       estKappa;                     //!< Defines the convergence factor for the estimator

  MOTOR_Type_e    motor_type;                   //!< Defines the motor type
  uint_least16_t  motor_numPolePairs;           //!< Defines the number of pole pairs for the motor

  float_t       motor_ratedFlux;              //!< Defines the rated flux of the motor, V/Hz
  float_t       motor_Rr;                     //!< Defines the rotor resistance, ohm
  float_t       motor_Rs;                     //!< Defines the stator resistance, ohm
  float_t       motor_Ls_d;                   //!< Defines the direct stator inductance, H
  float_t       motor_Ls_q;                   //!< Defines the quadrature stator inductance, H
  float_t       maxCurrent;                   //!< Defines the maximum current value, A
  float_t       maxCurrent_resEst;            //!< Defines the maximum current value for resistance estimation, A
  float_t       maxCurrent_indEst;            //!< Defines the maximum current value for inductance estimation, A
  float_t       maxCurrentSlope;              //!< Defines the maximum current slope for Id current trajectory
  float_t       maxCurrentSlope_powerWarp;    //!< Defines the maximum current slope for Id current trajectory during PowerWarp
  float_t       IdRated;                      //!< Defines the Id rated current value, A
  float_t       IdRatedFraction_indEst;       //!< Defines the fraction of Id rated current to use during inductance estimation
  float_t       IdRatedFraction_ratedFlux;    //!< Defines the fraction of Id rated current to use during rated flux estimation
  float_t       IdRated_delta;                //!< Defines the Id rated delta current value, A
  float_t       fluxEstFreq_Hz;               //!< Defines the flux estimation frequency, Hz

  uint_least32_t  ctrlWaitTime[CTRL_numStates]; //!< Defines the wait times for each controller state, estimator ticks
  uint_least32_t  estWaitTime[EST_numStates];   //!< Defines the wait times for each estimator state, estimator ticks
  uint_least32_t  FluxWaitTime[EST_Flux_numStates]; //!< Defines the wait times for each Ls estimator, estimator ticks
  uint_least32_t  LsWaitTime[EST_Ls_numStates]; //!< Defines the wait times for each Ls estimator, estimator ticks
  uint_least32_t  RsWaitTime[EST_Rs_numStates]; //!< Defines the wait times for each Rs estimator, estimator ticks
  uint_least32_t  ctrlFreq_Hz;                  //!< Defines the controller frequency, Hz
  uint_least32_t  estFreq_Hz;                   //!< Defines the estimator frequency, Hz
  uint_least32_t  RoverL_estFreq_Hz;            //!< Defines the R/L estimation frequency, Hz
  uint_least32_t  trajFreq_Hz;                  //!< Defines the trajectory frequency, Hz

  float_t       ctrlPeriod_sec;               //!< Defines the controller execution period, sec

  float_t       maxNegativeIdCurrent_a;       //!< Defines the maximum negative current that the Id PID is allowed to go to, A

  USER_ErrorCode_e  errorCode;

  float_t       bus_current_sf;                   //!< Defines the current scale factor for the system
  uint_least8_t   numBusCurrentSensors;            //!< Defines the number of current sensors

} USER_Params;


// **************************************************************************
// the functions


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _USERPARAMS_H_ definition
