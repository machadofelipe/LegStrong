#ifndef _CADENCE_H_
#define _CADENCE_H_
//! \file   cadence.h
//! \brief  Reads and calculates the cadence
//!

// **************************************************************************
// the includes

#include "types.h"
#include "IQmathLib.h"


// **************************************************************************
// the defines
#define CADENCE_Vars_INIT { 0, 0, 0, 0, 0.0 }

#define RPM_CONSTANT        		60000.0
#define MIN_CAD_PULSE_COUNTER   	2

namespace cadence {

    // **************************************************************************
    // the typedefs

    typedef struct _Vars_t_
    {
        uint32_t    PrevPulseTime;
        uint32_t    RisingEdgeTime;

        uint32_t    PulsePeriod;
        uint8_t     PulseCounter;
        _iq         kRPM;

    }Vars_t;


    // **************************************************************************
    // the globals

    extern volatile Vars_t gVars;


    // **************************************************************************
    // the functions

    //! \brief      readRPM
    void readRPM();

}; // cadence


#endif // !_CADENCE_H_
