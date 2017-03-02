//! \file   Mode01.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "mod01.h"
#include <stdlib.h>


// **************************************************************************
// the defines

using namespace elm327;


// **************************************************************************
// the globals


// **************************************************************************
// the functions

mod01::mod01() :
        m_gMod01Vars(),
        _pidMap(),
        _responseMsg()
{

    _pidMap[elm327::Mode01_PIDs_01__20] = &mod01::PIDs_01__20;
    _pidMap[elm327::Calculated_engine_load] = &mod01::Calculated_engine_load;
    //_pidMap[elm327::Engine_coolant_temperature] = &mod01::Engine_coolant_temperature;
    _pidMap[elm327::Engine_RPM] = &mod01::Engine_RPM;
    _pidMap[elm327::Vehicle_speed] = &mod01::Vehicle_speed;
    _pidMap[elm327::Throttle_position] = &mod01::Throttle_position;
    _pidMap[elm327::OBD_standards_this_vehicle_conforms_to] = &mod01::OBD_standards_this_vehicle_conforms_to;
    _pidMap[elm327::Run_time_since_engine_start] = &mod01::Run_time_since_engine_start;
    _pidMap[elm327::Mode01_PIDs_21__40] = &mod01::PIDs_21__40;
    _pidMap[elm327::Mode01_PIDs_41__60] = &mod01::PIDs_41__60;
    _pidMap[elm327::Control_module_voltage] = &mod01::Control_module_voltage;
    _pidMap[elm327::Fuel_Type] = &mod01::Fuel_Type;
    _pidMap[elm327::Mode01_PIDs_61__80] = &mod01::PIDs_61__80;
    _pidMap[elm327::Driver_demand_engine_torque] = &mod01::Driver_demand_engine_torque;
    _pidMap[elm327::Actual_engine_torque] = &mod01::Actual_engine_torque;
    _pidMap[elm327::Engine_reference_torque] = &mod01::Engine_reference_torque;

    init();
}

void mod01::init()
{

    for ( std::map< int, MOD01_HANDLER >::iterator itPidMap = _pidMap.begin();
            itPidMap != _pidMap.end();
            ++itPidMap )
    {
        if (itPidMap->first > Mode01_PIDs_01__20 && itPidMap->first <= Mode01_PIDs_21__40)
        {
            m_gMod01Vars.Mode01_PIDs_01__20 += ((unsigned long)1 << (Mode01_PIDs_21__40 - itPidMap->first));
        }

        else if (itPidMap->first > Mode01_PIDs_21__40 && itPidMap->first <= Mode01_PIDs_41__60)
        {
            m_gMod01Vars.Mode01_PIDs_21__40 += ((unsigned long)1 << (Mode01_PIDs_41__60 - itPidMap->first));
        }

        else if (itPidMap->first > Mode01_PIDs_41__60 && itPidMap->first <= Mode01_PIDs_61__80)
        {
            m_gMod01Vars.Mode01_PIDs_41__60 += ((unsigned long)1 << (Mode01_PIDs_61__80 - itPidMap->first));
        }

        else if (itPidMap->first > Mode01_PIDs_61__80)
        {
            m_gMod01Vars.Mode01_PIDs_61__80 += ((unsigned long)1 << (128 - itPidMap->first));
        }
    }

    m_gMod01Vars.OBD_standards_this_vehicle_conforms_to = 5;  // Not OBD compliant
    m_gMod01Vars.Fuel_Type = 8;   // Electric

    return;
}

void mod01::processPid(const std::string &pidString, std::string &responseMsg)
{

    _responseMsg = "";
    _responseMsg += pidString;

    int pidNumber = (int) strtol(pidString.c_str(), NULL, 16);
    std::map< int, MOD01_HANDLER >::iterator itPidMap = _pidMap.find(pidNumber);
    if (itPidMap != _pidMap.end())
    {
        (this->*itPidMap->second)();
        responseMsg += _responseMsg;
    }
    else
    {
        // Response for pid not in the list
    }

	return;
}


// !!!!!!!!!!!!!!!! Below code is still under optimization  !!!!!!!!!!!!!!!!!!!!!!!!


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
void mod01::PIDs_01__20()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Mode01_PIDs_01__20);

    return;
}

void mod01::Calculated_engine_load()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Calculated_engine_load);

    return;
}

//void mod01::Engine_coolant_temperature()
//{
//    utility::printHex(_responseMsg, m_gMod01Vars.Engine_coolant_temperature);
//
//    return;
//}

void mod01::Engine_RPM()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Engine_RPM);

    return;
}

void mod01::Vehicle_speed()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Vehicle_speed);

    return;
}

void mod01::Throttle_position()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Throttle_position);

    return;
}

void mod01::OBD_standards_this_vehicle_conforms_to()
{
    utility::printHex(_responseMsg, m_gMod01Vars.OBD_standards_this_vehicle_conforms_to);

    return;
}

void mod01::Run_time_since_engine_start()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Run_time_since_engine_start);

    return;
}

void mod01::PIDs_21__40()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Mode01_PIDs_21__40);

    return;
}

void mod01::PIDs_41__60()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Mode01_PIDs_41__60);

    return;
}

void mod01::Control_module_voltage()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Control_module_voltage);

    return;
}

void mod01::Fuel_Type()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Fuel_Type);

    return;
}

void mod01::PIDs_61__80()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Mode01_PIDs_61__80);

    return;
}

void mod01::Driver_demand_engine_torque()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Driver_demand_engine_torque);

    return;
}

void mod01::Actual_engine_torque()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Actual_engine_torque);

    return;
}

void mod01::Engine_reference_torque()
{
    utility::printHex(_responseMsg, m_gMod01Vars.Engine_reference_torque);

    return;
}

