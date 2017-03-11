#ifndef _SPEED_H_
#define _SPEED_H_
//! \file   speed.h
//! \brief  Variables for speed calculation use
//!

// **************************************************************************
// the includes

#include "types.h"


// **************************************************************************
// the defines
#define SPEED_Vars_INIT     { 0, 0, 0, 0.0 }

#define KMH_CONSTANT        (3.6)*1000.0

namespace speed {

    // **************************************************************************
    // the typedefs

    typedef struct _Vars_t_
    {
        uint32_t    MotorPrevPulseTime;

        uint32_t    MotorPulsePeriod;
        uint8_t     MotorPulseCounter;
        float       Kmh;

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
