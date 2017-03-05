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

std::string msgBuf;
bool newMessage = false;

// Clock timer
uint16_t gUpTimeSeconds = 0;

volatile cadence::Vars_t cadence::gVars = CADENCE_Vars_INIT;
volatile speed::Vars_t speed::gVars = SPEED_Vars_INIT;

elm327::mode01::Vars_t elm327::mode01::gVariables = MODE01_Vars_INIT;
elm327::mode08::Vars_t elm327::mode08::gVariables = MODE08_Vars_INIT;
elm327::mode09::Vars_t elm327::mode09::gVariables = MODE09_Vars_INIT;
elm327::mode21::Vars_t elm327::mode21::gVariables = MODE21_Vars_INIT;

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

#ifdef F2802xF
#ifdef __cplusplus
#pragma DATA_SECTION("rom_accessed_data");
#else
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif
#endif
HAL_Handle halHandle;

#ifdef F2802xF
#ifdef __cplusplus
#pragma DATA_SECTION("rom_accessed_data");
#else
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif
#endif
USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
#ifdef F2802xF
#ifdef __cplusplus
#pragma DATA_SECTION("rom_accessed_data");
#else
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif
#endif
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
//extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;


// **************************************************************************
// the functions

void main(void)
{

  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  InitFlash();
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

  elm327::mode01::init();
  elm327::mode09::init();


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
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


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


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
  // turn on the DRV8305 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8305 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif



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
//                    gMotorVars.Flag_Run_Identify = false;
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

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif


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
//    gMotorVars.Flag_Run_Identify = false;
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
//  // toggle status LED
//  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
//  {
//    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
//    gLEDcnt = 0;
//  }


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

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);
  elm327::mode01::gVariables.Engine_RPM = (uint16_t) ( _IQtoF(gMotorVars.Speed_krpm) * 4000.0 );

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);
  elm327::mode01::gVariables.Engine_reference_torque = (uint16_t) ( _IQtoF(gMotorVars.Torque_Nm) * 1000.0 );
  elm327::mode01::gVariables.Actual_engine_torque = (uint8_t) ( 125.0 + ( _IQtoF( _IQmpy( gMotorVars.Torque_Nm, (100.0/USER_MOTOR_MAX_TORQUE) ) ) ) );

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

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
  elm327::mode01::gVariables.Control_module_voltage = (uint16_t) ( _IQtoF(gMotorVars.VdcBus_kV) * 1000000.0 );

  // Get the DC buss current
  gMotorVars.IdcBus = _IQmpy(gAdcData.iBus,_IQ(USER_ADC_MAX_POSITIVE_BUS_CURRENT_A));

  elm327::mode01::gVariables.Calculated_engine_load = (uint8_t) ( _IQtoF(gMotorVars.IdcBus) * _IQtoF(gMotorVars.VdcBus_kV) * (100000.0/USER_MOTOR_MAX_POWER) );

  elm327::mode01::gVariables.Run_time = gUpTimeSeconds;

  elm327::mode01::gVariables.Vehicle_speed = speed::gVars.Kmh;

  elm327::mode01::gVariables.Driver_demand_engine_torque = (uint8_t) ( 125.0 + ( _IQtoF( _IQmpy( gMotorVars.IqRef_A, _IQ(100.0/USER_IQ_FULL_SCALE_CURRENT_A) ) ) ) );

  //  gMotorVars.Flag_Run_Identify = elm327::mode08::gVariables.Switch_Run;
  if (elm327::mode01::gVariables.Control_module_voltage < elm327::mode08::gVariables.Battery_cutout)
  {
      elm327::mode08::gVariables.Switch_Run = false;
  }


  return;
} // end of updateGlobalVariables_motor() function


void updateIqRef(CTRL_Handle handle)
{

    // Check this formula to regulate the IqRef based on cadence:
    //Setpoint = (-.0237*pow(((double)cad/300)-110, 2) + 100)*3.5;
    // assumes cadence in rpm and output is power (convert to frequency/current)

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
    gUpTimeSeconds++;

    return;
} // end of timer0ISR() function


//TODO: define the halhandle as extern and have the interruption in cadence.cpp?
__interrupt void cadencePulseISR(void)
{
    if (!GPIO_getData(halHandle->gpioHandle, GPIO_Number_6))
    {
        // compute the waveform period
        uint32_t timeCapture = HAL_readTimerCnt(halHandle, 2);

        // Read if it is in Normal Direction
        if ( (timeCapture + cadence::gVars.PrevPulseTime) < (2 * cadence::gVars.RisingEdgeTime) ) // off_period < on_period
        {
            cadence::gVars.PulsePeriod += (cadence::gVars.PrevPulseTime - timeCapture);
            cadence::gVars.PulseCounter++;
        }
        cadence::gVars.PrevPulseTime = timeCapture;

    }
    else
    {
        cadence::gVars.RisingEdgeTime = HAL_readTimerCnt(halHandle, 2);
    }

    HAL_acqCadencePulseInt(halHandle);
}

__interrupt void motorPulseISR(void)
{
    // compute the waveform period
    uint32_t timeCapture = HAL_readTimerCnt(halHandle, 2);

    speed::gVars.MotorPulsePeriod += (speed::gVars.MotorPrevPulseTime - timeCapture);
    speed::gVars.MotorPulseCounter++;

    speed::gVars.MotorPrevPulseTime = timeCapture;

    HAL_acqMotorPulseInt(halHandle);
}

//@} //defgroup
// end of file
