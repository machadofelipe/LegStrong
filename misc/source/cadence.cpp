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
        //TODO: make this math using IQ lib?
        gVars.kRPM = _IQ( (double)(60000.0 * gVars.PulseCounter) /
                (gVars.PulsePeriod * elm327::mode08::gVariables.PAS_magnets) );
        gVars.kRPM = (gVars.kRPM > _IQ(0)) && (gVars.kRPM < _IQ(0.220)) ? gVars.kRPM : _IQ(0);
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
            cadence::gVars.PulsePeriod += (cadence::gVars.PrevPulseTime - timeCapture);
            cadence::gVars.PulseCounter++;
        }
        cadence::gVars.PrevPulseTime = timeCapture;

    }
    else
    {
        cadence::gVars.RisingEdgeTime = HAL_readTimerCnt(&hal, 2);
    }

    HAL_acqCadencePulseInt(&hal);
}
