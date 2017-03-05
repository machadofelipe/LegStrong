//! \file   mode21.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mode21.h"
#include "utility.h"

#include <stdlib.h>
#include <map>


// **************************************************************************
// the defines

using namespace elm327::mode21;
using namespace utility;


// **************************************************************************
// the globals

bool createMode21Map(std::map< int, std::pair <int, void*> > &m)
{
//    m[Battery_temperature]      = std::make_pair(UINT8, &gVariables.Battery_temperature);
    m[Battery_current]          = std::make_pair(UINT16, &gVariables.Battery_current);
    m[Battery_current_avg]      = std::make_pair(UINT16, &gVariables.Battery_current);
    m[Battery_current_max]      = std::make_pair(UINT16, &gVariables.Battery_current_max);
    m[Battery_voltage]          = std::make_pair(UINT16, &gVariables.Battery_voltage);
    m[Battery_voltage_min]      = std::make_pair(UINT16, &gVariables.Battery_voltage_min);
    m[Rear_wheel_speed]         = std::make_pair(UINT16, &gVariables.Rear_wheel_speed);
    m[Rear_wheel_speed_avg]     = std::make_pair(UINT16, &gVariables.Rear_wheel_speed_avg);
    m[Cadence_RPM]              = std::make_pair(UINT16, &gVariables.Cadence_RPM);
    m[Cadence_RPM_avg]          = std::make_pair(UINT16, &gVariables.Cadence_RPM_avg);
    m[Cadence_RPM_max]          = std::make_pair(UINT16, &gVariables.Cadence_RPM_max);

    m[Battery_power]            = std::make_pair(UINT16, &gVariables.Battery_power);
    m[Battery_power_max]        = std::make_pair(UINT16, &gVariables.Battery_power_max);
    m[Battery_capacity_used]    = std::make_pair(UINT16, &gVariables.Battery_capacity_used);
    m[Energy_used]              = std::make_pair(UINT16, &gVariables.Energy_used);
    m[Battery_SOC]              = std::make_pair(UINT16, &gVariables.Battery_SOC);
    m[Trip_distance]            = std::make_pair(UINT16, &gVariables.Trip_distance);
    m[Trip_Time]                = std::make_pair(UINT16, &gVariables.Trip_Time);
    m[Energy_mileage]           = std::make_pair(UINT16, &gVariables.Energy_mileage);
    m[Status]                   = std::make_pair(UINT8, &gVariables.Status);

//    m[Battery_cycles]           = std::make_pair(UINT16, &gVariables.Battery_cycles);
//    m[Odometer]                 = std::make_pair(UINT16, &gVariables.Odometer);

    return true;
}

static std::map< int, std::pair <int, void*> > m_map;
static bool _dummy = createMode21Map(m_map);


// **************************************************************************
// the functions


void ::elm327::mode21::processPid(const std::string &pidString, std::string &responseMsg)
{
    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, std::pair <int, void*> >::iterator itPidMap = m_map.find(pidNumber);

    switch (itPidMap->second.first)
    {
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

