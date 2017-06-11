//! \file   cadence.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "cadence.h"
#include "hal.h"
#include "mode08.h"


// **************************************************************************
// the defines
using namespace cadence;


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void ::cadence::readRPM() {

    DISABLE_INTERRUPTS;
    {
        gVars.kRPM = gVars.PulseCounter < MIN_CAD_PULSE_COUNTER ? _IQ(0) :
                _IQ( (double) (RPM_CONSTANT * gVars.PulseCounter) /
                              (gVars.PulsePeriod * elm327::mode08::gVariables.PAS_magnets) );

        gVars.PulsePeriod = 0;
        gVars.PulseCounter = 0;
    }
    ENABLE_INTERRUPTS;

}

__interrupt void cadencePulseISR(void)
{
    if (!GPIO_getData(hal.gpioHandle, GPIO_Number_6))
    {
        // compute the waveform period
        uint32_t timeCapture = HAL_readTimerCnt(&hal, 2);

        // Read if it is in Normal Direction
        if ( (timeCapture + cadence::gVars.PrevPulseTime) < (2 * cadence::gVars.RisingEdgeTime) ) // off_period < on_period
        {
            gVars.PulsePeriod += (cadence::gVars.PrevPulseTime - timeCapture);
            gVars.PulseCounter++;
        }
        else
        {
            gVars.PulsePeriod = 0;
            gVars.PulseCounter = 0;
        }
        gVars.PrevPulseTime = timeCapture;

    }
    else
    {
        gVars.RisingEdgeTime = HAL_readTimerCnt(&hal, 2);
    }

    HAL_acqCadencePulseInt(&hal);
}
