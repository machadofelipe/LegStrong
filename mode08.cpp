//! \file   mode08.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mode08.h"
#include <stdlib.h>


// **************************************************************************
// the defines

using namespace elm327::mode08;


// **************************************************************************
// the globals

struct MODE08{
    static std::map< int, MODE08_HANDLER > createMap()
    {
        std::map< int, MODE08_HANDLER > m;

        m[SW_Run] = &swRunCall;
        m[TOG_PowerMode] = &togPowerModeCall;

        return m;
    }
    static std::map< int, MODE08_HANDLER > _pidMap;

};

std::map< int, MODE08_HANDLER > MODE08::_pidMap = MODE08::createMap();

std::string _responseMsg;

// **************************************************************************
// the functions

void ::elm327::mode08::processPid(const std::string &pidString, std::string &responseMsg)
{

    _responseMsg = "";
    _responseMsg += pidString;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MODE08_HANDLER >::iterator itPidMap = MODE08::_pidMap.find(pidNumber);
    if (itPidMap != MODE08::_pidMap.end())
    {
        (*itPidMap->second)();
        responseMsg += _responseMsg;
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
//!             PID number      01  02  03  04  05  06  07  08  09  0A  0B  0C  0D  0E  0F  10  11  12  13  14  15  16  17  18  19  1A  1B  1C  1D  1E  1F  20
//!
//!             So, supported PIDs are: 01, 03, 04, 05, 06, 07, 0C, 0D, 0E, 0F, 10, 11, 13, 15, 1C, 1F and 20

//! \brief      PIDs_01__20
//void mode08::PIDs_01__20()
//{
//    utility::printHex(_responseMsg, m_Mode08Vars.Mode08_PIDs_01__20);
//
//    return;
//}

void ::elm327::mode08::swRunCall()
{
    gVariables.SW_Run ^= true;
    utility::printHex(_responseMsg, gVariables.SW_Run);

    return;
}

void ::elm327::mode08::togPowerModeCall()
{
    gVariables.TOG_PowerMode = gVariables.TOG_PowerMode < 2 ?
            (gVariables.TOG_PowerMode + 1) : 0;
    utility::printHex(_responseMsg, gVariables.TOG_PowerMode);

    return;
}

//void mode08::PIDs_21__40()
//{
//    utility::printHex(_responseMsg, m_gMode08Vars.Mode08_PIDs_21__40);
//
//    return;
//}
//
//void mode08::PIDs_41__60()
//{
//    utility::printHex(_responseMsg, m_gMode08Vars.Mode08_PIDs_41__60);
//
//    return;
//}
//
//void mode08::PIDs_61__80()
//{
//    utility::printHex(_responseMsg, m_gMode08Vars.Mode08_PIDs_61__80);
//
//    return;
//}

