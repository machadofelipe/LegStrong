//! \file   Mode01.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "Mode01.h"
#include <map>


// **************************************************************************
// the defines


// **************************************************************************
// the globals

std::string* MOD01_responseMsg;

MOTOR_Vars_t* MOD01_gMotorsVarsPtr;

struct MOD01{
    static std::map< int, MOD01_HANDLER > createMap()
    {
        std::map< int, MOD01_HANDLER > m;

        m[PIDs_01__20] = &MOD01_PIDs_01__20;
        m[Engine_coolant_temperature] = &MOD01_Engine_coolant_temperature;
        m[Engine_RPM] = &MOD01_Engine_RPM;
        m[Vehicle_speed] = &MOD01_Vehicle_speed;
        m[Throttle_position] = &MOD01_Throttle_position;
        m[OBD_standards_this_vehicle_conforms_to] = &MOD01_OBD_standards_this_vehicle_conforms_to;
        m[PIDs_21__40] = &MOD01_PIDs_21__40;
        m[PIDs_41__60] = &MOD01_PIDs_41__60;
        m[Fuel_Type] = &MOD01_Fuel_Type;
        m[PIDs_61__80] = &MOD01_PIDs_61__80;
        m[PIDs_81__A0] = &MOD01_PIDs_81__A0;
        m[PIDs_A1__C0] = &MOD01_PIDs_A1__C0;
        m[PIDs_C1__E0] = &MOD01_PIDs_C1__E0;

        return m;
    }
    static std::map< int, MOD01_HANDLER > pidMap;

};

std::map< int, MOD01_HANDLER > MOD01::pidMap = MOD01::createMap();

// **************************************************************************
// the functions

void MOD01_processPid(const std::string &pidString, std::string &responseMsg)
{

    MOD01_responseMsg = &responseMsg;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MOD01_HANDLER >::iterator itPidMap = MOD01::pidMap.find(pidNumber);
    if (itPidMap != MOD01::pidMap.end())
    {
        (*itPidMap->second)();
    }

	return;
}


//! \brief      Mode 1 PID 00
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
void MOD01_PIDs_01__20()
{
//    char buf[6];
    //TODO: maybe also keep a pointer to a function and transform the values just when it is needed?
//    sprintf(buf," %04x", *(m_pidMap16b.find(pid)->second));
    (*MOD01_responseMsg) = "4100188010";
    return;
}

void MOD01_Engine_coolant_temperature()
{
    (*MOD01_responseMsg) = "413C";
    return;
}


void MOD01_Engine_RPM()
{
    (*MOD01_responseMsg) = "410000";
    return;
}

void MOD01_Vehicle_speed()
{
    (*MOD01_responseMsg) = "4100";
    return;
}

void MOD01_Throttle_position()
{
    (*MOD01_responseMsg) = "4100";
    return;
}

void MOD01_OBD_standards_this_vehicle_conforms_to()
{
    (*MOD01_responseMsg) = "4105";
    return;
}

void MOD01_PIDs_21__40()
{
    (*MOD01_responseMsg) = "4100188000";
    return;
}

void MOD01_PIDs_41__60()
{
    (*MOD01_responseMsg) = "4100008000";
    return;
}

void MOD01_Fuel_Type()
{
    (*MOD01_responseMsg) = "4108";
    return;
}

void MOD01_PIDs_61__80()
{
    (*MOD01_responseMsg) = "4100000000";
    return;
}

void MOD01_PIDs_81__A0()
{
    (*MOD01_responseMsg) = "4100000000";
    return;
}

void MOD01_PIDs_A1__C0()
{
    (*MOD01_responseMsg) = "4100000000";
    return;
}

void MOD01_PIDs_C1__E0()
{
    (*MOD01_responseMsg) = "4100000000";
    return;
}
