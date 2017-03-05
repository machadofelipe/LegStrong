//! \file   mode08.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mode08.h"
#include <stdlib.h>
#include <map>


// **************************************************************************
// the defines

using namespace elm327::mode08;


// **************************************************************************
// the globals

struct mode08Pids{
    static std::map< int, MODE08_HANDLER > createMap()
    {
        std::map< int, MODE08_HANDLER > m;

        m[Switch_Run] = &swRun;
        m[Toggle_option] = &togOption;

        m[PAS_magnets] = &pasMagnets;
        m[Wheel_circ] = &wheelCirc;
        m[Throttle_ramp] = &throttleRamp;
        m[Throttle_mode] = &throttleMode;
        m[Battery_cutout] = &batteryCutout;
        m[Battery_limit] = &batteryLimit;
        m[Power_limit] = &powerLimit;
        m[Speed_limit] = &speedLimit;

        m[Increase_value] = &increaseValue;
        m[Decresase_value] = &decreaseValue;
        m[Reset_all] = &resetAll;

        return m;
    }
    static std::map< int, MODE08_HANDLER > m_map;

};

std::map< int, MODE08_HANDLER > mode08Pids::m_map = mode08Pids::createMap();

std::string _mode08Msg;

// **************************************************************************
// the functions

void ::elm327::mode08::processPid(const std::string &pidString, std::string &responseMsg)
{

    _mode08Msg = "";
    _mode08Msg += pidString;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MODE08_HANDLER >::iterator itPidMap = mode08Pids::m_map.find(pidNumber);
    if (itPidMap != mode08Pids::m_map.end())
    {
        (*itPidMap->second)();
        responseMsg += _mode08Msg;
    }
    else
    {
        // Response for pid not in the list
    }

	return;
}


//! \brief      Mode 8 PID 00
//! \details    A request for this PID returns 4 bytes of data. Each bit, from MSB to LSB, represents one of the next 32 PIDs and is giving information about if it is supported.
//!
//!             For example, if the car response is BE1FA813, it can be decoded like this:
//!
//!             Hexadecimal     B               E               1               F               A               8               1               3
//!             Binary          1   0   1   1   1   1   1   0   0   0   0   1   1   1   1   1   1   0   1   0   1   0   0   0   0   0   0   1   0   0   1   1
//!             Supported?      Yes No  Yes Yes Yes Yes Yes No  No  No  No  Yes Yes Yes Yes Yes Yes No  Yes No  Yes No  No  No  No  No  No  Yes No  No  Yes Yes
//!             PID number      01  02  03  04  05  06  07  08  08  0A  0B  0C  0D  0E  0F  10  11  12  13  14  15  16  17  18  19  1A  1B  1C  1D  1E  1F  20
//!
//!             So, supported PIDs are: 01, 03, 04, 05, 06, 07, 0C, 0D, 0E, 0F, 10, 11, 13, 15, 1C, 1F and 20

//! \brief      PIDs_01__20
//void mode08::PIDs_01__20()
//{
//    utility::printHex(_mode08Msg, m_Mode08Vars.Mode08_PIDs_01__20);
//
//    return;
//}

void ::elm327::mode08::swRun()
{
    gVariables.Switch_Run ^= true;

    utility::printHex(_mode08Msg, gVariables.Switch_Run);

    return;
}

void ::elm327::mode08::togOption()
{
    gVariables.Toggle_option = gVariables.Toggle_option < (Options_END - 1) ?
            (gVariables.Toggle_option + 1) : Options_START;

    utility::printHex(_mode08Msg, gVariables.Toggle_option);

    return;
}

//void mode08::PIDs_21__40()
//{
//    utility::printHex(_mode08Msg, m_gMode08Vars.Mode08_PIDs_21__40);
//
//    return;
//}

void ::elm327::mode08::pasMagnets()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == PAS_magnets)
    {
        utility::printHex(_mode08Msg, gVariables.PAS_magnets);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::wheelCirc()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Wheel_circ)
    {
        utility::printHex(_mode08Msg, gVariables.Wheel_circ);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::throttleRamp()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Throttle_ramp)
    {
        utility::printHex(_mode08Msg, gVariables.Throttle_ramp);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::throttleMode()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Throttle_mode)
    {
        utility::printHex(_mode08Msg, gVariables.Throttle_mode);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::batteryCutout()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Battery_cutout)
    {
        utility::printHex(_mode08Msg, gVariables.Battery_cutout);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::batteryLimit()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Battery_limit)
    {
        utility::printHex(_mode08Msg, gVariables.Battery_limit);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::powerLimit()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Power_limit)
    {
        utility::printHex(_mode08Msg, gVariables.Power_limit);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}

void ::elm327::mode08::speedLimit()
{
    if (gVariables.Toggle_option == Options_START || gVariables.Toggle_option == Speed_limit)
    {
        utility::printHex(_mode08Msg, gVariables.Speed_limit);
    }
    else
    {
        _mode08Msg = "";
    }

    return;
}


//void ::elm327::mode08::mode08::PIDs_41__60()
//{
//    utility::printHex(_mode08Msg, m_gMode08Vars.Mode08_PIDs_41__60);
//
//    return;
//}

//TODO: protect MAX and MIN
void ::elm327::mode08::increaseValue()
{
    switch(gVariables.Toggle_option)
    {
        case PAS_magnets:
            gVariables.PAS_magnets += 1;
            break;

        case Wheel_circ:
            gVariables.Wheel_circ += 5;
            break;

        case Throttle_ramp:
            gVariables.Throttle_ramp += 1;
            break;

        case Throttle_mode:
            gVariables.Throttle_mode += 1;
            break;

        case Battery_cutout:
            gVariables.Battery_cutout += 1;
            break;

        case Battery_limit:
            gVariables.Battery_limit += 1;
            break;

        case Power_limit:
            gVariables.Power_limit += 50;
            break;

        case Speed_limit:
            gVariables.Speed_limit += 5;
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
            gVariables.PAS_magnets -= 1;
            break;

        case Wheel_circ:
            gVariables.Wheel_circ -= 5;
            break;

        case Throttle_ramp:
            gVariables.Throttle_ramp -= 1;
            break;

        case Throttle_mode:
            gVariables.Throttle_mode -= 1;
            break;

        case Battery_cutout:
            gVariables.Battery_cutout -= 1;
            break;

        case Battery_limit:
            gVariables.Battery_limit -= 1;
            break;

        case Power_limit:
            gVariables.Power_limit -= 50;
            break;

        case Speed_limit:
            gVariables.Speed_limit -= 5;
            break;

        default:
            break;
    }
}

void ::elm327::mode08::resetAll()
{
//    gVariables = MODE08_Vars_INIT;
}


//void mode08::PIDs_61__80()
//{
//    utility::printHex(_mode08Msg, m_gMode08Vars.Mode08_PIDs_61__80);
//
//    return;
//}

