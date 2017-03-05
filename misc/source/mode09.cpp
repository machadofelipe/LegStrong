//! \file   mode09.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mode09.h"
#include <stdlib.h>
#include <map>


// **************************************************************************
// the defines

using namespace elm327::mode09;


// **************************************************************************
// the globals


struct mode09Pids{
    static std::map< int, MODE09_HANDLER > createMap()
    {
        std::map< int, MODE09_HANDLER > m;

        m[PIDs_01__20] = &pids01_20;
        m[Vehicle_Identification_Number] = &vehicleId;
        m[Calibration_ID] = &calibrationId;
        m[ECU_name] = &ecuName;

        return m;
    }
    static std::map< int, MODE09_HANDLER > m_map;

};

std::map< int, MODE09_HANDLER > mode09Pids::m_map = mode09Pids::createMap();

std::string _mode09Msg;

// **************************************************************************
// the functions

void ::elm327::mode09::init()
{

    for ( std::map< int, MODE09_HANDLER >::iterator itPidMap = mode09Pids::m_map.begin();
            itPidMap != mode09Pids::m_map.end();
            ++itPidMap )
    {
        if (itPidMap->first > PIDs_01__20)
        {
            gVariables.PIDs_01__20 += ((unsigned long)1 << (32 - itPidMap->first));
        }
    }
}

void ::elm327::mode09::processPid(const std::string &pidString, std::string &responseMsg)
{

    _mode09Msg = "";
    _mode09Msg += pidString;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MODE09_HANDLER >::iterator itPidMap = mode09Pids::m_map.find(pidNumber);
    if (itPidMap != mode09Pids::m_map.end())
    {
        (*itPidMap->second)();
        responseMsg += _mode09Msg;
    }
    else
    {
        // Response for pid not in the list
    }

    return;
}


//! \brief      Mode 9 PID 00
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
void ::elm327::mode09::pids01_20()
{
    utility::printHex(_mode09Msg, gVariables.PIDs_01__20);

    return;
}

void ::elm327::mode09::vehicleId()
{
    _mode09Msg += gVariables.Vehicle_Identification_Number;

    return;
}

void ::elm327::mode09::calibrationId()
{
    _mode09Msg += gVariables.Calibration_ID;

    return;
}

void ::elm327::mode09::ecuName()
{
    _mode09Msg += gVariables.ECU_name;

    return;
}
