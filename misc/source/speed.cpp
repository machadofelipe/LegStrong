//! \file   speed.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "speed.h"
#include "hal.h"
#include "mode08.h"


// **************************************************************************
// the defines
using namespace speed;


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void ::speed::readKmh() {

    DISABLE_INTERRUPTS;
    {

        if (gVars.MotorPulseCounter < MIN_SPEED_PULSE_COUNTER)
        {
            gVars.Kmh =  _IQ(0.0);
        }
        else
        {
            gVars.Kmh = _IQ ( (double) ( KPH_CONSTANT * elm327::mode08::gVariables.Wheel_circ * gVars.MotorPulseCounter ) /
                                        ( gVars.MotorPulsePeriod * elm327::mode08::gVariables.Wheel_magnets ) );

            //TODO: fix issues with the approximations
            gVars.Distance += _IQ ( (double) ( KPS_CONSTANT * elm327::mode08::gVariables.Wheel_circ * gVars.MotorPulseCounter ) /
                    ( gVars.MotorPulsePeriod * elm327::mode08::gVariables.Wheel_magnets ) );
        }

        gVars.MotorPulsePeriod = 0;
        gVars.MotorPulseCounter = 0;
    }
    ENABLE_INTERRUPTS;

}

//TODO: find the issue with the speed (probally in the speed interruption
__interrupt void motorPulseISR(void)
{
    // compute the waveform period
    uint32_t timeCapture = HAL_readTimerCnt(&hal, 2);

    speed::gVars.MotorPulsePeriod += (speed::gVars.MotorPrevPulseTime - timeCapture);
    speed::gVars.MotorPulseCounter++;

    speed::gVars.MotorPrevPulseTime = timeCapture;

    HAL_acqMotorPulseInt(&hal);
}
