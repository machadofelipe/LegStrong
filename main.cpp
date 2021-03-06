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

//! \file   main.cpp
//! \brief  Using InstaSPIN-FOC only as a torque controller
//!
//! (C) Copyright 2011, Texas Instruments, Inc.
//! \defgroup PROJ_LAB04_OVERVIEW Project Overview
//!
//! Running InstaSPIN-FOC only as a Torque controller
//!

// **************************************************************************
// the includes

// system includes
#include <ctype.h>
#include "main.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals

//TODO: optimize:
std::string msgBuf;
bool newMessage = false;
uint16_t gUpTimeSeconds = 0;  // set to runtime directly ?
uint32_t  gPrevPulseTime = 0;
uint_least16_t gCounter_updateGlobals = 0;
bool Flag_Latch_softwareUpdate = true;
//END optimize


elm327::mode01::Vars_t elm327::mode01::gVariables = MODE01_Vars_INIT;
elm327::mode08::Vars_t elm327::mode08::gVariables = MODE08_Vars_INIT;
elm327::mode09::Vars_t elm327::mode09::gVariables = MODE09_Vars_INIT;
elm327::mode21::Vars_t elm327::mode21::gVariables = MODE21_Vars_INIT;


#ifdef F2802xF
#ifdef __cplusplus
#pragma DATA_SECTION("rom_accessed_data");
#else
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif
#endif
HAL_Obj* halHandle;

#ifdef F2802xF
#ifdef __cplusplus
#pragma DATA_SECTION("rom_accessed_data");
#else
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif
#endif
USER_Params gUserParams;

#ifdef F2802xF
#ifdef __cplusplus
#pragma DATA_SECTION("rom_accessed_data");
#else
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif
#endif
CTRL_Obj ctrl;				//v1p7 format


volatile cadence::Vars_t cadence::gVars = CADENCE_Vars_INIT;

volatile speed::Vars_t speed::gVars = SPEED_Vars_INIT;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;


// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;

CTRL_Handle ctrlHandle;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

_iq gFlux_pu_to_Wb_sf;
_iq gFlux_pu_to_VpHz_sf;
_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;
_iq gTorque_Flux_Iq_pu_to_Nm_sf;


// **************************************************************************
// the functions

void main(void)
{

  // Only used if running from FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  InitFlash();


  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);

  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
  uint_least8_t estNumber = 0; //v1p7 format
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default


  {
	CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion.targetProc = version.targetProc;
    gMotorVars.CtrlVersion.rsvd = version.rsvd;
    gMotorVars.CtrlVersion.major = version.major;
    gMotorVars.CtrlVersion.minor = version.minor;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);

  // setup faults
  HAL_setupFaults(halHandle);

  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);

  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);

  // enable Sci interrupt
  HAL_enableSciInt(halHandle);

  // enable global interrupts
  HAL_enableGlobalInts(halHandle);

  // enable debug interrupts
  HAL_enableDebugInt(halHandle);

  // disable the PWM
  HAL_disablePwm(halHandle);

  // enable the Timer 0 interrupts
  HAL_enableTimer0Int(halHandle);

  // enable the Cadence Pulse interrupts
  HAL_enableCadencePulseInt(halHandle);

  // enable the Cadence Pulse interrupts
  HAL_enableMotorPulseInt(halHandle);

  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);


  // initialize the communication objects and its variables
  elm327::mode01::init();
  elm327::mode09::init();


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();


  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys));

    // Dis-able the Library internal PI.  Iq has no reference now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, elm327::mode08::gVariables.Switch_Run);

            if(flag_ctrlStateChanged)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));
                      HAL_setBias(halHandle,HAL_SensorType_BusCurrent,0,_IQ(I_BUS_offset));

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                    }

                    // Return the bias value for currents
                    gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
                    gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
                    gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);
                    gMotorVars.Idc_bias = HAL_getBias(halHandle,HAL_SensorType_BusCurrent,0);

                    // Return the bias value for voltages
                    gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
                    gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
                    gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);

                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    elm327::mode08::gVariables.Switch_Run = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }

              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;
              USER_calcPIgains(ctrlHandle);
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        readSensorsCallback();
        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandle);
          }

        // update Iq reference
        updateIqRef(ctrlHandle);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

        HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);
        HAL_readDrvData(halHandle,&gDrvSpi8301Vars);


        if(newMessage)
        {
            // Check received data
            Message msgObj = Message(msgBuf);
            msgObj.HandleMessage(msgBuf);
            HAL_SciASendMessage( halHandle, msgBuf.c_str() );
            msgBuf = "";
            newMessage = false;
        }

      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    elm327::mode08::gVariables.Switch_Run = false;

  } // end of for(;;) loop

} // end of main() function


#ifdef FLASH
#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#else
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif
#endif
interrupt void mainISR(void)
{

  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);

  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);

  // run the controller
  CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData);

  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);

  // setup the controller
  CTRL_setup(ctrlHandle);

  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    // Calculations done using C28x IQmath Library
    // WARNING: avoid using float/double calculations as much as possible

    // get the motor speed estimate
    gMotorVars.Speed_krpm = elm327::mode08::gVariables.Switch_Run ? EST_getSpeed_krpm(obj->estHandle) : _IQ(0.0);
    elm327::mode21::gVariables.Motor_kRPM = gMotorVars.Speed_krpm > _IQ(0.0) ? (uint16_t) _IQtoIQ15(gMotorVars.Speed_krpm) : _IQ15(0.0);
    //TODO: engine RPM calculations are not optimal, try IQ Math
    elm327::mode01::gVariables.Engine_RPM = _IQ2( (float) (_IQ15toF(elm327::mode21::gVariables.Motor_kRPM)*1000.0) );

    // get the torque estimate
    gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);
    elm327::mode01::gVariables.Engine_reference_torque = (uint16_t) (_IQint(gMotorVars.Torque_Nm));
    elm327::mode01::gVariables.Actual_engine_torque = (uint8_t) ( 125 + ( _IQ21int( _IQ21mpy( _IQtoIQ21(gMotorVars.Torque_Nm), _IQ21(100.0/USER_MOTOR_MAX_TORQUE) ) ) ) );
    elm327::mode01::gVariables.Driver_demand_engine_torque = (uint8_t) ( 125 + ( _IQ21int( _IQ21mpy( _IQtoIQ21(gMotorVars.IqRef_A), _IQ21(100.0/USER_IQ_FULL_SCALE_CURRENT_A) ) ) ) );

    // get the magnetizing current
    gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

    // get the rotor resistance
    gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

    // get the stator resistance
    gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

    // get the stator inductance in the direct coordinate direction
    gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

    // get the stator inductance in the quadrature coordinate direction
    gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

    // get the flux in V/Hz in floating point
    gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

    // get the flux in Wb in fixed point
    gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

    // get the controller state
    gMotorVars.CtrlState = CTRL_getState(handle);

    // get the estimator state
    gMotorVars.EstState = EST_getState(obj->estHandle);

    // read Vd and Vq vectors per units
    gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
    gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

    // calculate vector Vs in per units
    gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

    // read Id and Iq vectors in amps
    gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
    gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

    // calculate vector Is in amps
    gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));

    // Get the DC buss voltage
    elm327::mode01::gVariables.Control_module_voltage = (uint16_t) ( _IQtoF(gMotorVars.Vdc) * 1000.0 );
    elm327::mode21::gVariables.Battery_voltage = (uint16_t) _IQtoIQ10(gMotorVars.Vdc);

    if (_IQ10int(elm327::mode21::gVariables.Battery_voltage) < elm327::mode08::gVariables.Battery_cutout)
    {
        elm327::mode08::gVariables.Switch_Run = false;
    }

    // Calculate the engine load 0-100%
    elm327::mode01::gVariables.Calculated_engine_load = (uint8_t) _IQ20int( _IQ20mpy( _IQ20mpy(_IQtoIQ20(gMotorVars.Is_A), _IQtoIQ20(gMotorVars.Vdc)),
                                                                                      _IQ20(100.0/USER_MOTOR_MAX_POWER)                                   ));

    // Get the DC bus current
    elm327::mode21::gVariables.Battery_current = (uint16_t) _IQtoIQ11(gMotorVars.Idc + _IQ(8)); // Add the max. negative current

    // Calculate the Battery power output in Watts
    elm327::mode21::gVariables.Battery_power = (uint16_t) ( _IQ5mpy(_IQtoIQ5(gMotorVars.Idc), _IQtoIQ5(gMotorVars.Vdc)) + _IQ5 (512) ); // Add the max. negative power

    // Get the speed and trip distance
    elm327::mode01::gVariables.Vehicle_speed = (uint8_t) _IQint(speed::gVars.Kmh);
    elm327::mode21::gVariables.Rear_wheel_speed = (uint16_t) _IQtoIQ10(speed::gVars.Kmh);
    elm327::mode21::gVariables.Trip_distance = (uint16_t) _IQtoIQ8(speed::gVars.Distance);

    // Get the Cadence
    elm327::mode21::gVariables.Cadence_RPM = (uint16_t) _IQ8mpy( _IQtoIQ8(cadence::gVars.kRPM), _IQ8(1000) );

    // Get the battery resistance and SoC
    if (gMotorVars.Idc < _IQ(1))
    {
        gMotorVars.Vdc_v0 = gMotorVars.Vdc;

        // TODO: To be optimized with IQ Math and support different batteries (equations are good for LG 18650 MJ1)
        if (gMotorVars.Vdc_v0 > _IQ(3.25*BATTERY_CELLS))
        {
            elm327::mode21::gVariables.Battery_SOC = (uint16_t) _IQ9( ( ( ( ( ( (double) _IQtoF(gMotorVars.Vdc_v0)) / BATTERY_CELLS ) - 3.25 ) / ( 4.15 - 3.25 ) ) * 90.0 ) + 10.0 );
        }
        else if (gMotorVars.Vdc_v0 > _IQ(2.7*BATTERY_CELLS))
        {
            elm327::mode21::gVariables.Battery_SOC = (uint16_t) _IQ9( ( ( ( ( (double) _IQtoF(gMotorVars.Vdc_v0)) / BATTERY_CELLS ) - 2.70 ) / ( 3.25 - 2.7 ) ) * 10.0 );
        }
        else
        {
            elm327::mode21::gVariables.Battery_SOC = (uint16_t) _IQ9(0);
        }
    }
    // TODO: Need new math calculation
    else if (gMotorVars.Idc > _IQ(2))
    {
        elm327::mode21::gVariables.Battery_resistance = _IQ6( (float) (1000.0 * _IQtoF(gMotorVars.Vdc_v0 - gMotorVars.Vdc)) / _IQtoF(gMotorVars.Idc));
    }

    // From readSensorsCallback
    elm327::mode21::gVariables.Battery_capacity_used = (gMotorVars.mAh) >> (17-2);
    elm327::mode21::gVariables.Energy_used = (gMotorVars.kWh) >> (30-16);

    // From timer while running
    elm327::mode01::gVariables.Run_time = gUpTimeSeconds;

    return;
} // end of updateGlobalVariables_motor() function

void readSensorsCallback() {

    // Read voltage and current (if CTRL already has Idc_bias calculate)
    gMotorVars.Vdc = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V));
    gMotorVars.Idc = gMotorVars.Idc_bias ? _IQmpy(gAdcData.iBus,_IQ(USER_IQ_FULL_SCALE_BUS_CURRENT_A)) : _IQ(0.0);

    // compute the period
    uint32_t timeCapture = HAL_readTimerCnt(&hal, 2);
    int32_t readPeriod = gPrevPulseTime - timeCapture;

    //Protected against timer overflow
    if (gPrevPulseTime > timeCapture)
    {
        gMotorVars.mAh += _IQ17mpy( _IQtoIQ17(gMotorVars.Idc) , _IQ17(readPeriod/(60.0*60.0*1000.0)) );

        gMotorVars.kWh += _IQ30( _IQtoF(gMotorVars.Idc) * _IQtoF(gMotorVars.Vdc) * (readPeriod/(60.0*60.0*1000.0*1000.0*1000.0)) );
    }

    gPrevPulseTime = timeCapture;

}

void updateIqRef(CTRL_Handle handle)
{

    // Controller setpoint:
    _iq IqRef_pu;

    //TODO: FIX magic numbers / create MACRO for the equations
    //TODO: Implement a boost button for hill climbing
    // Switches on only while cadence > 10
    // Full power while cadence > 10
    // Back to previous mode after candence = 0
    switch (elm327::mode08::gVariables.Throttle_ramp)
    {
        //TODO: maybe remove the weak in future
        case WEAK_THROTTLE:
            IqRef_pu = _IQmpy( _IQ(14), (cadence::gVars.kRPM - _IQ(0.040)) );
            break;

        case MEDIUM_THROTTLE:
            IqRef_pu = _IQ( 1 - ( 237 * ( _IQtoF(cadence::gVars.kRPM) - 0.110) * (_IQtoF(cadence::gVars.kRPM) - 0.110) ) );
            break;

        case STRONG_THROTTLE:
            IqRef_pu = (_IQtoF(cadence::gVars.kRPM) > 0.040) ?
                        _IQmpy( _IQ(0.25), _IQ( log((double) (_IQtoF(cadence::gVars.kRPM)*1000 - 39)) ) ) : _IQ(0);
            break;

        case BOOST_THROTTLE:
            IqRef_pu = (_IQtoF(cadence::gVars.kRPM) > 0.020) ?
                        _IQ(elm327::mode08::gVariables.Battery_limit) : _IQ(0);
            break;

        default:
            IqRef_pu = _IQ(0);
            break;
    }

    gMotorVars.IqRef_A = IqRef_pu > _IQ(0) ?
            _IQmpy( _IQ(elm327::mode08::gVariables.Battery_limit), IqRef_pu ) : _IQ(0);
    gMotorVars.IqRef_A = IqRef_pu > _IQ(1) ?
            _IQ(elm327::mode08::gVariables.Battery_limit) : gMotorVars.IqRef_A;


  _iq iq_ref = _IQmpy(gMotorVars.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));

  // set the speed reference so that the forced angle rotates in the correct direction for startup
  if(_IQabs(gMotorVars.Speed_krpm) < _IQ(0.01))
    {
      if(iq_ref < _IQ(0.0))
        {
          CTRL_setSpd_ref_krpm(handle,_IQ(-0.01));
        }
      else if(iq_ref > _IQ(0.0))
        {
          CTRL_setSpd_ref_krpm(handle,_IQ(0.01));
        }
    }

  // Set the Iq reference that use to come out of the PI speed control
  CTRL_setIq_ref_pu(handle, iq_ref);

  return;
} // end of updateIqRef() function

//TODO: optimize Message class and add this function to its code
__interrupt void sciaRxFifoISR(void)
{
char rdataA;    // Received data for SCI-A

    // TODO: while?
    if(SCI_getRxFifoStatus(halHandle->sciAHandle) != SCI_FifoLevel_Empty)
    {
		// Read data
		rdataA = SCI_read(halHandle->sciAHandle);

        if(rdataA == '\r')
        {
            newMessage = true;
        }
        else if (rdataA > 47 && !newMessage)
        {
            msgBuf.push_back( toupper(rdataA) );
        }

    }

    // Clear Overflow flag
    SCI_clearRxFifoOvf(halHandle->sciAHandle);
    // Clear Interrupt flag
    SCI_clearRxFifoInt(halHandle->sciAHandle);
    // Issue PIE ack
    PIE_clearInt(halHandle->pieHandle, PIE_GroupNumber_9);

    return;
}

__interrupt void timer0ISR(void)
{

    cadence::readRPM();
    speed::readKmh();

    // acknowledge the Timer 0 interrupt
    HAL_acqTimer0Int(halHandle);

    // increment up-time
    if (elm327::mode08::gVariables.Switch_Run)
    {
        gUpTimeSeconds++;

    		if (elm327::mode08::gVariables.Boost_timer)
    		{
    			elm327::mode08::gVariables.Boost_timer--;
    		}
    		else if(elm327::mode08::gVariables.Throttle_ramp == BOOST_THROTTLE)
    		{
    			elm327::mode08::setBoost(false);
    		}
    }

    return;
} // end of timer0ISR() function


//@} //defgroup
// end of file
