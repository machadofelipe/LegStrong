//! \file   mode08.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mode08.h"
#include "utility.h"

#include <stdlib.h>
#include <map>


// **************************************************************************
// the defines

using namespace elm327::mode08;
using namespace utility;


// **************************************************************************
// the globals

namespace elm327
{
    namespace mode08
    {

        bool createMap(std::map< int, std::pair <int, void*> > &m)
        {
            m[Switch_Run] = std::make_pair(FN_PTR, (void*) &swRun);
            m[Toggle_option] = std::make_pair(FN_PTR, (void*) &togOption);

            m[PAS_magnets] = std::make_pair(UINT8, &gVariables.PAS_magnets);
            m[Wheel_magnets] = std::make_pair(UINT8, &gVariables.Wheel_magnets);
            m[Wheel_circ] = std::make_pair(UINT16, &gVariables.Wheel_circ);
            m[Throttle_ramp] = std::make_pair(UINT8, &gVariables.Throttle_ramp);
//            m[Throttle_mode] = std::make_pair(BOOL, &gVariables.Throttle_mode);
            m[Battery_cutout] = std::make_pair(UINT8, &gVariables.Battery_cutout);
            m[Battery_limit] = std::make_pair(UINT8, &gVariables.Battery_limit);
    //        m[Power_limit] = std::make_pair(UINT16, &gVariables.Power_limit);
    //        m[Speed_limit] = std::make_pair(UINT8, &gVariables.Speed_limit);

            m[Increase_value] = std::make_pair(FN_PTR, (void*) &increaseValue);
            m[Decresase_value] = std::make_pair(FN_PTR, (void*) &decreaseValue);
            m[Reset_all] = std::make_pair(FN_PTR, (void*) &resetAll);

            return true;
        }

        static std::map< int, std::pair <int, void*> > m_map;
        static bool _dummy = createMap(m_map);
    }
}

// **************************************************************************
// the functions

void ::elm327::mode08::processPid(const std::string &pidString, std::string &responseMsg)
{
    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, std::pair <int, void*> >::iterator itPidMap = m_map.find(pidNumber);

    switch (itPidMap->second.first)
    {
        case UINT16:
            if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == itPidMap->first)
            {
                responseMsg += pidString;
                printHex( responseMsg, (*(uint16_t*) itPidMap->second.second) );
            }
            break;

        case UINT8:
            if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == itPidMap->first)
            {
                responseMsg += pidString;
                printHex( responseMsg, (*(uint8_t*) itPidMap->second.second) );
            }
            break;

        case BOOL:
            if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == itPidMap->first)
            {
                responseMsg += pidString;
                printHex( responseMsg, (*(bool*) itPidMap->second.second) );
            }
            break;

        case FN_PTR:
            ((funcptr) itPidMap->second.second)();
            break;

        default:
            break;
    }

    return;
}

void ::elm327::mode08::swRun()
{
    gVariables.Toggle_option = Options_START;
    gVariables.Switch_Run ^= true;

    return;
}

void ::elm327::mode08::togOption()
{

    if (!gVariables.Switch_Run)
    {
        gVariables.Toggle_option = gVariables.Toggle_option < (Options_END - 1) ?
                (gVariables.Toggle_option + 1) : Options_START;
    }

    return;
}

void ::elm327::mode08::increaseValue()
{
    switch(gVariables.Toggle_option)
    {
        case PAS_magnets:
            gVariables.PAS_magnets = gVariables.PAS_magnets < MAX_PAS_MAGNETS ?
                    (gVariables.PAS_magnets + 1) : MIN_PAS_MAGNETS;
            break;

        case Wheel_magnets:
            gVariables.Wheel_magnets = gVariables.Wheel_magnets < MAX_WHEEL_MAGNETS ?
                    (gVariables.Wheel_magnets + 1) : MIN_WHEEL_MAGNETS;
            break;

        case Wheel_circ:
            gVariables.Wheel_circ = gVariables.Wheel_circ < MAX_WHEEL_CIRC ?
                    (gVariables.Wheel_circ + WHEEL_CIRC_STEPS) : MIN_WHEEL_CIRC;;
            break;

        case Throttle_ramp:
            gVariables.Throttle_ramp = gVariables.Throttle_ramp < MAX_THROTTLE_RAMP ?
                    (gVariables.Throttle_ramp + 1) : MIN_THROTTLE_RAMP;
            break;

        case Battery_cutout:
            gVariables.Battery_cutout = gVariables.Battery_cutout < MAX_BAT_CUTOUT ?
                    (gVariables.Battery_cutout + 1) : MIN_BAT_CUTOUT;
            break;

        case Battery_limit:
            gVariables.Battery_limit = gVariables.Battery_limit < MAX_BAT_LIMIT ?
                    (gVariables.Battery_limit + 1) : MIN_BAT_LIMIT;
            break;

        default:
            break;
    }
}

void ::elm327::mode08::decreaseValue()
{
    switch(gVariables.Toggle_option)
    {
        case PAS_magnets:
            gVariables.PAS_magnets = gVariables.PAS_magnets > MIN_PAS_MAGNETS ?
                    (gVariables.PAS_magnets - 1) : MAX_PAS_MAGNETS;
            break;

        case Wheel_magnets:
            gVariables.Wheel_magnets = gVariables.Wheel_magnets > MIN_WHEEL_MAGNETS ?
                    (gVariables.Wheel_magnets - 1) : MAX_WHEEL_MAGNETS;
            break;

        case Wheel_circ:
            gVariables.Wheel_circ = gVariables.Wheel_circ > MIN_WHEEL_CIRC ?
                    (gVariables.Wheel_circ - WHEEL_CIRC_STEPS) : MAX_WHEEL_CIRC;;
            break;

        case Throttle_ramp:
            gVariables.Throttle_ramp = gVariables.Throttle_ramp > MIN_THROTTLE_RAMP ?
                    (gVariables.Throttle_ramp - 1) : MAX_THROTTLE_RAMP;
            break;

        case Battery_cutout:
            gVariables.Battery_cutout = gVariables.Battery_cutout > MIN_BAT_CUTOUT ?
                    (gVariables.Battery_cutout - 1) : MAX_BAT_CUTOUT;
            break;

        case Battery_limit:
            gVariables.Battery_limit = gVariables.Battery_limit > MIN_BAT_LIMIT ?
                    (gVariables.Battery_limit - 1) : MAX_BAT_LIMIT;
            break;

        default:
            break;
    }
}

void ::elm327::mode08::resetAll()
{
    Vars_t defaultValues = MODE08_Vars_INIT;
    gVariables = defaultValues;
}


