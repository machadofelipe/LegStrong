//! \file   Mode01.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "Mode01.h"
#include <map>
//#include <stdio.h>
#include <stdlib.h>


// **************************************************************************
// the defines


// **************************************************************************
// the globals

std::string* MOD01_responseMsg;



struct MOD01{
    static std::map< int, MOD01_HANDLER > MOD01_createMap()
    {
        std::map< int, MOD01_HANDLER > m;

        m[Mode01_PIDs_01__20] = &MOD01_PIDs_01__20;
        m[Calculated_engine_load] = &MOD01_Calculated_engine_load;
        m[Engine_coolant_temperature] = &MOD01_Engine_coolant_temperature;
        m[Engine_RPM] = &MOD01_Engine_RPM;
        m[Vehicle_speed] = &MOD01_Vehicle_speed;
        m[Throttle_position] = &MOD01_Throttle_position;
        m[OBD_standards_this_vehicle_conforms_to] = &MOD01_OBD_standards_this_vehicle_conforms_to;
        m[Mode01_PIDs_21__40] = &MOD01_PIDs_21__40;
        m[Mode01_PIDs_41__60] = &MOD01_PIDs_41__60;
        m[Control_module_voltage] = &MOD01_Control_module_voltage;
        m[Fuel_Type] = &MOD01_Fuel_Type;
        m[Mode01_PIDs_61__80] = &MOD01_PIDs_61__80;
        m[Driver_demand_engine_torque] = &MOD01_Driver_demand_engine_torque;
        m[Actual_engine_torque] = &MOD01_Actual_engine_torque;
        m[Engine_reference_torque] = &MOD01_Engine_reference_torque;

        return m;
    }
    static std::map< int, MOD01_HANDLER > pidMap;

};

std::map< int, MOD01_HANDLER > MOD01::pidMap = MOD01::MOD01_createMap();

// **************************************************************************
// the functions

void MOD01_init(Mode01_Vars_t &gMod01Vars)
{

    for ( std::map< int, MOD01_HANDLER >::iterator itPidMap = MOD01::pidMap.begin();
            itPidMap != MOD01::pidMap.end();
            ++itPidMap )
    {
        if (itPidMap->first > Mode01_PIDs_01__20 && itPidMap->first <= Mode01_PIDs_21__40)
        {
            gMod01Vars.Mode01_PIDs_01__20 += ((unsigned long)1 << (Mode01_PIDs_21__40 - itPidMap->first));
        }

        else if (itPidMap->first > Mode01_PIDs_21__40 && itPidMap->first <= Mode01_PIDs_41__60)
        {
            gMod01Vars.Mode01_PIDs_21__40 += ((unsigned long)1 << (Mode01_PIDs_41__60 - itPidMap->first));
        }

        else if (itPidMap->first > Mode01_PIDs_41__60 && itPidMap->first <= Mode01_PIDs_61__80)
        {
            gMod01Vars.Mode01_PIDs_41__60 += ((unsigned long)1 << (Mode01_PIDs_61__80 - itPidMap->first));
        }

        else if (itPidMap->first > Mode01_PIDs_61__80)
        {
            gMod01Vars.Mode01_PIDs_61__80 += ((unsigned long)1 << (128 - itPidMap->first));
        }
    }

    gMod01Vars.OBD_standards_this_vehicle_conforms_to = 5;  // Not OBD compliant
    gMod01Vars.Fuel_Type = 8;   // Electric

}

void MOD01_processPid(const std::string &pidString, std::string &responseMsg, Mode01_Vars_t &gMod01Vars)
{

    responseMsg += pidString;
    MOD01_responseMsg = &responseMsg;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MOD01_HANDLER >::iterator itPidMap = MOD01::pidMap.find(pidNumber);
    if (itPidMap != MOD01::pidMap.end())
    {
        (*itPidMap->second)(gMod01Vars);
    }
    else
    {
        // Response for pid not in the list
    }

	return;
}

void MOD01_printf(std::string &string, uint32_t &value)
{
    for (int i=7; i >= 0; i--)
    {
        string += (MOD01_hexToASCII[((value >> 4*i) & 0x0000000F)]);
    }

    return;
}

void MOD01_printf(std::string &string, uint16_t &value)
{
    for (int i=3; i >= 0; i--)
    {
        string += (MOD01_hexToASCII[((value >> 4*i) & 0x000F)]);
    }

    return;
}


void MOD01_printf(std::string &string, uint8_t &value)
{
    for (int i=1; i >= 0; i--)
    {
        string += (MOD01_hexToASCII[((value >> 4*i) & 0x0F)]);
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
void MOD01_PIDs_01__20(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Mode01_PIDs_01__20);

    return;
}

void MOD01_Calculated_engine_load(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Calculated_engine_load);

    return;
}

void MOD01_Engine_coolant_temperature(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Engine_coolant_temperature);

    return;
}

void MOD01_Engine_RPM(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Engine_RPM);

    return;
}

void MOD01_Vehicle_speed(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Vehicle_speed);

    return;
}

void MOD01_Throttle_position(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Throttle_position);

    return;
}

void MOD01_OBD_standards_this_vehicle_conforms_to(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.OBD_standards_this_vehicle_conforms_to);

    return;
}

void MOD01_PIDs_21__40(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Mode01_PIDs_21__40);

    return;
}

void MOD01_PIDs_41__60(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Mode01_PIDs_41__60);

    return;
}

void MOD01_Control_module_voltage(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Control_module_voltage);

    return;
}

void MOD01_Fuel_Type(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Fuel_Type);

    return;
}

void MOD01_PIDs_61__80(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Mode01_PIDs_61__80);

    return;
}

void MOD01_Driver_demand_engine_torque(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Driver_demand_engine_torque);

    return;
}

void MOD01_Actual_engine_torque(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Actual_engine_torque);

    return;
}

void MOD01_Engine_reference_torque(Mode01_Vars_t &gMod01Vars)
{
    MOD01_printf((*MOD01_responseMsg), gMod01Vars.Engine_reference_torque);

    return;
}

