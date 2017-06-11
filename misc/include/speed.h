#ifndef _SPEED_H_
#define _SPEED_H_
//! \file   speed.h
//! \brief  Variables for speed calculation use
//!

// **************************************************************************
// the includes

#include "types.h"
#include "IQmathLib.h"


// **************************************************************************
// the defines
#define SPEED_Vars_INIT     { 0, 0, 0, _IQ(0.0), _IQ(0.0) }

#define KPH_CONSTANT        			3600.0
#define KPS_CONSTANT        			1.0
#define MIN_SPEED_PULSE_COUNTER   	1

namespace speed {

    // **************************************************************************
    // the typedefs

    typedef struct _Vars_t_
    {
        uint32_t    MotorPrevPulseTime;

        uint32_t    MotorPulsePeriod;
        uint8_t     MotorPulseCounter;
        _iq         Kmh;
        _iq         Distance;

    }Vars_t;


    // **************************************************************************
    // the globals

    extern volatile Vars_t gVars;


    // **************************************************************************
    // the functions

    //! \brief      readRPM
    void readKmh();

}; // speed


#endif // !_SPEED_H_
