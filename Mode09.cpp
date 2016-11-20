//! \file   Mode09.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "Mode09.h"
#include <map>
#include <stdlib.h>


// **************************************************************************
// the defines


// **************************************************************************
// the globals

std::string* MOD09_responseMsg;

Mode09_Vars_t gMod09Vars = MOD09_Vars_INIT;


struct MOD09{
    static std::map< int, MOD09_HANDLER > MOD09_createMap()
    {
        std::map< int, MOD09_HANDLER > m;

        m[Mode09_PIDs_01__20] = &MOD09_PIDs_01__20;
        m[Vehicle_Identification_Number] = &MOD09_Vehicle_Identification_Number;
        m[Calibration_ID] = &MOD09_Calibration_ID;
        m[ECU_name] = &MOD09_ECU_name;

        return m;
    }
    static std::map< int, MOD09_HANDLER > pidMap;

};

std::map< int, MOD09_HANDLER > MOD09::pidMap = MOD09::MOD09_createMap();

// **************************************************************************
// the functions

void MOD09_init()
{

    for ( std::map< int, MOD09_HANDLER >::iterator itPidMap = MOD09::pidMap.begin();
            itPidMap != MOD09::pidMap.end();
            ++itPidMap )
    {
        if (itPidMap->first > Mode09_PIDs_01__20)
        {
            gMod09Vars.Mode09_PIDs_01__20 += ((unsigned long)1 << (32 - itPidMap->first));
        }
    }
}

void MOD09_processPid(const std::string &pidString, std::string &responseMsg)
{

    responseMsg += pidString;
    MOD09_responseMsg = &responseMsg;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MOD09_HANDLER >::iterator itPidMap = MOD09::pidMap.find(pidNumber);
    if (itPidMap != MOD09::pidMap.end())
    {
        (*itPidMap->second)();
    }
    else
    {
        // Response for pid not in the list
    }

    return;
}

void MOD09_printf(std::string &string, uint32_t &value)
{
    for (int i=7; i >= 0; i--)
    {
        string += (MOD09_hexToASCII[((value >> 4*i) & 0x0000000F)]);
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
void MOD09_PIDs_01__20()
{
    MOD09_printf((*MOD09_responseMsg), gMod09Vars.Mode09_PIDs_01__20);

    return;
}

void MOD09_Vehicle_Identification_Number()
{
    (*MOD09_responseMsg) += gMod09Vars.Vehicle_Identification_Number;

    return;
}

void MOD09_Calibration_ID()
{
    (*MOD09_responseMsg) += gMod09Vars.Calibration_ID;

    return;
}

void MOD09_ECU_name()
{
    (*MOD09_responseMsg) += gMod09Vars.ECU_name;

    return;
}
