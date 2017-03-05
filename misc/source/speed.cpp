//! \file   speed.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "speed.h"
#include "hal.h"


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
        //TODO: make this math using IQ lib?
        gVars.Kmh = (KMH_CONSTANT*gVars.MotorPulseCounter)/gVars.MotorPulsePeriod;
        gVars.MotorPulsePeriod = 0;
        gVars.MotorPulseCounter = 0;
    }
    ENABLE_INTERRUPTS;

}
