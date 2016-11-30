#ifndef _MOD01_H_
#define _MOD01_H_
//! \file   mod01.h
//! \brief  Show current data
//!

// **************************************************************************
// the includes

#include "types.h"
#include "utility.h"

#include <string>
#include <map>


// **************************************************************************
// the defines
#define MOD01_Vars_INIT { 0, 0, 0, 0, 0, 0, 0,      \
                          0,                        \
                          0, 0, 0,                  \
                          0, 0, 0, 0,               }


namespace elm327 {

    // **************************************************************************
    // the typedefs

    //! \brief      Enumeration Mode 01 - Show current data.
    //! \details    Standard OBD-II PIDs as defined by SAE J197.
    typedef enum
    {
        Mode01_PIDs_01__20 = 0x00,
        Calculated_engine_load = 0x04,
        Engine_coolant_temperature = 0x05,
        Engine_RPM = 0x0C,
        Vehicle_speed = 0x0D,
        Throttle_position = 0x11,
        OBD_standards_this_vehicle_conforms_to = 0x1C,
    //    Run_time_since_engine_start = 0x1F,

        Mode01_PIDs_21__40 = 0x20,
    //    Fuel_Tank_Level_Input = 0x2F,

        Mode01_PIDs_41__60 = 0x40,
        Control_module_voltage = 0x42,
        Fuel_Type = 0x51,
    //    Engine_fuel_rate = 0x5E,

        Mode01_PIDs_61__80 = 0x60,
        Driver_demand_engine_torque = 0x61,
        Actual_engine_torque = 0x62,
        Engine_reference_torque = 0x63,
    } Mode01_PIDs_e;


    typedef struct _Mode01_Vars_t_
    {
        uint32_t    Mode01_PIDs_01__20;
        uint8_t     Calculated_engine_load;
        uint8_t     Engine_coolant_temperature;
        uint16_t    Engine_RPM;
        uint8_t     Vehicle_speed;
        uint8_t     Throttle_position;
        uint8_t     OBD_standards_this_vehicle_conforms_to;
    //    uint16_t    Run_time_since_engine_start;

        uint32_t    Mode01_PIDs_21__40;
    //    uint8_t     Fuel_Tank_Level_Input;

        uint32_t    Mode01_PIDs_41__60;
        uint16_t    Control_module_voltage;
        uint8_t     Fuel_Type;
    //    uint16_t    Engine_fuel_rate;

        uint32_t    Mode01_PIDs_61__80;
        uint8_t     Driver_demand_engine_torque;
        uint8_t     Actual_engine_torque;
        uint16_t    Engine_reference_torque;

    }Mode01_Vars_t;


    // **************************************************************************
    // the class

    class mod01 {

        typedef void (elm327::mod01::*MOD01_HANDLER)();

        public:

            mod01();

            ~mod01() { };

            //! \brief      processPid
            //! \param[in]  pid
            //! \param[in]  responseMsg
            void processPid(const std::string &pid, std::string &responseMsg);

            Mode01_Vars_t m_gMod01Vars;


        private:

            void init();

            //! \brief      PIDs_01__20
            void PIDs_01__20();
            void Calculated_engine_load();
            void Engine_coolant_temperature();
            void Engine_RPM();
            void Vehicle_speed();
            void Throttle_position();
            void OBD_standards_this_vehicle_conforms_to();

            //! \brief      PIDs_21__40
            void PIDs_21__40();

            //! \brief      PIDs_41__60
            void PIDs_41__60();
            void Control_module_voltage();
            void Fuel_Type();

            //! \brief      PIDs_61__80
            void PIDs_61__80();
            void Driver_demand_engine_torque();
            void Actual_engine_torque();
            void Engine_reference_torque();


            std::map< int, MOD01_HANDLER > _pidMap;

            std::string _responseMsg;


    }; // mod01
}; //ELM327

// **************************************************************************
// PID Codes that are not used

//    Monitor_status = 0x01,
//    Fuel_system_status = 0x03,
//    Short_term_fuel_trim_Bank_1 = 0x06,
//    Long_term_fuel_trim_Bank_1 = 0x07,
//    Short_term_fuel_trim_Bank_2 = 0x08,
//    Long_term_fuel_trim_Bank_2 = 0x09,
//    Fuel_pressure_gauge_pressure = 0x0A,
//    Intake_manifold_absolute_pressure = 0x0B,
//    Timing_advance = 0x0E,
//    Intake_air_temperature = 0x0F,
//    MAF_air_flow_rate = 0x10,
//    Commanded_secondary_air_status = 0x12,
//    Oxygen_sensors_present_in_2_banks = 0x13,
//    Oxygen_Sensor_1_A_Voltage_B_Short_term_fuel_trim = 0x14,
//    Oxygen_Sensor_2_A_Voltage_B_Short_term_fuel_trim = 0x15,
//    Oxygen_Sensor_3_A_Voltage_B_Short_term_fuel_trim = 0x16,
//    Oxygen_Sensor_4_A_Voltage_B_Short_term_fuel_trim = 0x17,
//    Oxygen_Sensor_5_A_Voltage_B_Short_term_fuel_trim = 0x18,
//    Oxygen_Sensor_6_A_Voltage_B_Short_term_fuel_trim = 0x19,
//    Oxygen_Sensor_7_A_Voltage_B_Short_term_fuel_trim = 0x1A,
//    Oxygen_Sensor_8_A_Voltage_B_Short_term_fuel_trim = 0x1B,
//    Oxygen_sensors_present_in_4_banks = 0x1D,
//    Auxiliary_input_status = 0x1E,
//    Distance_traveled_with_MIL = 0x21,
//    Fuel_Rail_Pressure = 0x22,
//    Fuel_Rail_Gauge_Pressure = 0x23,
//    Oxygen_Sensor_1_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x24,
//    Oxygen_Sensor_2_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x25,
//    Oxygen_Sensor_3_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x26,
//    Oxygen_Sensor_4_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x27,
//    Oxygen_Sensor_5_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x28,
//    Oxygen_Sensor_6_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x29,
//    Oxygen_Sensor_7_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x2A,
//    Oxygen_Sensor_8_AB_Fuel_Air_Equivalence_Ratio_CD_Voltage = 0x2B,
//    Commanded_EGR = 0x2C,
//    EGR_Error = 0x2D,
//    Commanded_evaporative_purge = 0x2E,
//    Warm_ups_since_codes_cleared = 0x30,
//    Distance_traveled_since_codes_cleared = 0x31,
//    Evap_System_Vapor_Pressure = 0x32,
//    Absolute_Barometric_Pressure = 0x33,
//    Oxygen_Sensor_1_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x34,
//    Oxygen_Sensor_2_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x35,
//    Oxygen_Sensor_3_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x36,
//    Oxygen_Sensor_4_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x37,
//    Oxygen_Sensor_5_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x38,
//    Oxygen_Sensor_6_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x39,
//    Oxygen_Sensor_7_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x3A,
//    Oxygen_Sensor_8_AB_Fuel_Air_Equivalence_Ratio_CD_Current = 0x3B,
//    Catalyst_Temperature_Bank_1_Sensor_1 = 0x3C,
//    Catalyst_Temperature_Bank_2_Sensor_1 = 0x3D,
//    Catalyst_Temperature_Bank_1_Sensor_2 = 0x3E,
//    Catalyst_Temperature_Bank_2_Sensor_2 = 0x3F,
//    Monitor_status_this_drive_cycle = 0x41,
//    Absolute_load_value = 0x43,
//    Fuel_Air_commanded_equivalence_ratio = 0x44,
//    Relative_throttle_position = 0x45,
//    Ambient_air_temperature = 0x46,
//    Absolute_throttle_position_B = 0x47,
//    Absolute_throttle_position_C = 0x48,
//    Accelerator_pedal_position_D = 0x49,
//    Accelerator_pedal_position_E = 0x4A,
//    Accelerator_pedal_position_F = 0x4B,
//    Commanded_throttle_actuator = 0x4C,
//    Time_run_with_MIL_on = 0x4D,
//    Time_since_trouble_codes_cleared = 0x4E,
//    Maximum_value_for_oxygen_sensor_and_intake_manifold_absolute_pressure = 0x4F,
//    Maximum_value_for_mass_air_flow_sensor = 0x50,
//    Ethanol_fuel = 0x52,
//    Absolute_Evap_system_Vapor_Pressure = 0x53,
//    Evap_system_vapor_pressure = 0x54,
//    Short_term_secondary_oxygen_sensor_trim_A_bank_1_B_bank_3 = 0x55,
//    Long_term_secondary_oxygen_sensor_trim_A_bank_1_B_bank_3 = 0x56,
//    Short_term_secondary_oxygen_sensor_trim_A_bank_2_B_bank_4 = 0x57,
//    Long_term_secondary_oxygen_sensor_trim_A_bank_2_B_bank_4 = 0x58,
//    Fuel_rail_absolute_pressure = 0x59,
//    Relative_accelerator_pedal_position = 0x5A,
//    Hybrid_battery_pack_remaining_life = 0x5B,
//    Engine_oil_temperature = 0x5C,
//    Fuel_injection_timing = 0x5D,
//    Emission_requirements_to_which_vehicle_is_designed = 0x5F,
//    Engine_torque_data = 0x64,
//    Auxiliary_IO_supported = 0x65,
//    Mass_air_flow_sensor = 0x66,
//    Engine_coolant_temperature_67 = 0x67,
//    Intake_air_temperature_sensor = 0x68,
//    Commanded_EGR_and_EGR_Error = 0x69,
//    Commanded_Diesel_intake_air_flow_control_and_relative_intake_air_flow_position = 0x6A,
//    Exhaust_gas_recirculation_temperature = 0x6B,
//    Commanded_throttle_actuator_control_and_relative_throttle_position = 0x6C,
//    Fuel_pressure_control_system = 0x6D,
//    Injection_pressure_control_system = 0x6E,
//    Turbocharger_compressor_inlet_pressure = 0x6F,
//    Boost_pressure_control = 0x70,
//    Variable_Geometry_turbo_VGT_control = 0x71,
//    Wastegate_control = 0x72,
//    Exhaust_pressure = 0x73,
//    Turbocharger_RPM = 0x74,
//    Turbocharger_temperature_1 = 0x75,
//    Turbocharger_temperature_2 = 0x76,
//    Charge_air_cooler_temperature_CACT = 0x77,
//    Exhaust_Gas_temperature_EGT_Bank_1 = 0x78,
//    Exhaust_Gas_temperature_EGT_Bank_2 = 0x79,
//    Diesel_particulate_filter_DPF_1 = 0x7A,
//    Diesel_particulate_filter_DPF_2 = 0x7B,
//    Diesel_Particulate_filter_DPF_temperature = 0x7C,
//    NOx_NTE_control_area_status = 0x7D,
//    PM_NTE_control_area_status = 0x7E,
//    Engine_run_time = 0x7F,
//    Engine_run_time_AECD_1 = 0x81,
//    Engine_run_time_AECD_2 = 0x82,
//    NOx_sensor = 0x83,
//    Manifold_surface_temperature = 0x84,
//    NOx_reagent_system = 0x85,
//    Particulate_matter_PM_sensor = 0x86,
//    Intake_manifold_absolute_pressure = 0x87,
//    Numerous_data = 0xC3,
//    Engine_requests = 0xC4


#endif // !_MOD01_H_
