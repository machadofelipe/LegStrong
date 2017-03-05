//! \file   cadence.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "cadence.h"
#include "hal.h"


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
        gVars.RPM = (10000000.0*gVars.PulseCounter)/gVars.PulsePeriod;
        gVars.PulsePeriod = 0;
        gVars.PulseCounter = 0;
    }
    ENABLE_INTERRUPTS;

}
