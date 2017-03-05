//! \file   mode01.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mode01.h"
#include "utility.h"

#include <stdlib.h>
#include <map>


// **************************************************************************
// the defines

using namespace elm327::mode01;
using namespace utility;


// **************************************************************************
// the globals

bool createMode01Map(std::map< int, std::pair <int, void*> > &m)
{
    m[PIDs_01__20]                  = std::make_pair(UINT32, &gVariables.PIDs_01__20);
    m[Monitor_status]               = std::make_pair(UINT32, &gVariables.Monitor_status);
    m[Calculated_engine_load]       = std::make_pair(UINT8, &gVariables.Calculated_engine_load);
    //m[Engine_coolant_temperature] = std::make_pair(UINT32, &gVariables.Engine_coolant_temperature
    m[Engine_RPM]                   = std::make_pair(UINT16, &gVariables.Engine_RPM);
    m[Vehicle_speed]                = std::make_pair(UINT8, &gVariables.Vehicle_speed);
    m[Throttle_position]            = std::make_pair(UINT8, &gVariables.Throttle_position);
    m[OBD_standards]                = std::make_pair(UINT8, &gVariables.OBD_standards);
    m[Run_time]                     = std::make_pair(UINT16, &gVariables.Run_time);
    m[PIDs_21__40]                  = std::make_pair(UINT32, &gVariables.PIDs_21__40);
    m[Fuel_tank_level_input]        = std::make_pair(UINT8, &gVariables.Fuel_tank_level_input);
    m[PIDs_41__60]                  = std::make_pair(UINT32, &gVariables.PIDs_41__60);
    m[Control_module_voltage]       = std::make_pair(UINT16, &gVariables.Control_module_voltage);
    m[Fuel_Type]                    = std::make_pair(UINT8, &gVariables.Fuel_Type);
    m[Engine_fuel_rate]             = std::make_pair(UINT16, &gVariables.Engine_fuel_rate);
    m[PIDs_61__80]                  = std::make_pair(UINT32, &gVariables.PIDs_61__80);
    m[Driver_demand_engine_torque]  = std::make_pair(UINT8, &gVariables.Driver_demand_engine_torque);
    m[Actual_engine_torque]         = std::make_pair(UINT8, &gVariables.Actual_engine_torque);
    m[Engine_reference_torque]      = std::make_pair(UINT16, &gVariables.Engine_reference_torque);

    return true;
}

static std::map< int, std::pair <int, void*> > m_map;
static bool _dummy = createMode01Map(m_map);


// **************************************************************************
// the functions

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
void ::elm327::mode01::init()
{
    // Fill all the PIDs supported (bit encoded) based on the Map
    for ( std::map< int, std::pair <int, void*> >::iterator itPidMap = m_map.begin();
            itPidMap != m_map.end();
            ++itPidMap )
    {
        if (itPidMap->first > PIDs_01__20 && itPidMap->first <= PIDs_21__40)
        {
            gVariables.PIDs_01__20 += ((unsigned long)1 << (PIDs_21__40 - itPidMap->first));
        }

        else if (itPidMap->first > PIDs_21__40 && itPidMap->first <= PIDs_41__60)
        {
            gVariables.PIDs_21__40 += ((unsigned long)1 << (PIDs_41__60 - itPidMap->first));
        }

        else if (itPidMap->first > PIDs_41__60 && itPidMap->first <= PIDs_61__80)
        {
            gVariables.PIDs_41__60 += ((unsigned long)1 << (PIDs_61__80 - itPidMap->first));
        }

        else if (itPidMap->first > PIDs_61__80)
        {
            gVariables.PIDs_61__80 += ((unsigned long)1 << (128 - itPidMap->first));
        }
    }

    gVariables.OBD_standards = 5;  // Not OBD compliant
    gVariables.Fuel_Type = 8;   // Electric

    return;
}

void ::elm327::mode01::processPid(const std::string &pidString, std::string &responseMsg)
{
    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, std::pair <int, void*> >::iterator itPidMap = m_map.find(pidNumber);

    switch (itPidMap->second.first)
    {
        case UINT32:
            responseMsg += pidString;
            printHex( responseMsg, (*(uint32_t*) itPidMap->second.second) );
            break;

        case UINT16:
            responseMsg += pidString;
            printHex( responseMsg, (*(uint16_t*) itPidMap->second.second) );
            break;

        case UINT8:
            responseMsg += pidString;
            printHex( responseMsg, (*(uint8_t*) itPidMap->second.second) );
            break;

        default:
            break;
    }

	return;
}

