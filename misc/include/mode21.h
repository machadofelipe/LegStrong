#ifndef _MODE21_H_
#define _MODE21_H_
//! \file   mode21.h
//! \brief  Extra data specifically of LegStrong
//!

// **************************************************************************
// the includes

#include "types.h"

#include <string>


// **************************************************************************
// the defines
#define MODE21_Vars_INIT { 0, 0, 0, 0, 0,      \
                           0, 0, 0, 0, 0, 0, 0, 0,        \
                                                                }

namespace elm327 {


    // **************************************************************************
    // the class

    namespace mode21 {

        // **************************************************************************
        // the typedefs

        //! \brief      Enumeration Mode 21 - Control operation of on-board component/system
        //! \details    Codes defined specifically for this application
        typedef enum
        {
            // Reserved: PIDs_01__20 = 0x00,
                // Direct acquired variables
                Battery_temperature = 0x01,
                Battery_current,
                Battery_current_avg,
                Battery_current_max,
                Battery_voltage,
                Battery_voltage_min,
                Rear_wheel_speed,
                Rear_wheel_speed_avg,
                Motor_RPM,
                Cadence_RPM,
                Cadence_RPM_avg,
                Cadence_RPM_max,

            // Reserved: PIDs_21__40 = 0x20,
                // Calculated variables
                Battery_power = 0x21,
                Battery_power_max,
                Battery_capacity_used,
                Energy_used,
                Battery_SOC,
                Battery_resistance,
                Trip_distance,
                Trip_Time,
                Energy_mileage,
                Status,

            // Reserved: PIDs_41__60 = 0x40,
                // Memory variables
                Battery_cycles = 0x41,
                Odometer,

            // Reserved: PIDs_61__80 = 0x60,

        }PIDs_e;


        typedef struct _Vars_t_
        {
            // Reserved: uint32_t    PIDs_01__20;
            // Direct acquired variable
//            uint8_t     Battery_temperature;       // Battery temperature, [ºC], [-40ºC - 215ºC], [A-40]
            uint16_t    Battery_current;            // Battery current, [A],  [0A - 32A], [(256*A+B)/2048]
//            uint16_t    Battery_current_avg;        // Battery current, [A],  [0A - 21.845A], [(256*A+B)/3000]
//            uint16_t    Battery_current_max;        // Battery current, [A],  [0A - 21.845A], [(256*A+B)/3000]
            uint16_t    Battery_voltage;            // Battery voltage, [V],  [0A - 64V], [(256*A+B)/1024]
//            uint16_t    Battery_voltage_min;        // Battery voltage, [V],  [0A - 65.535V], [(256*A+B)/1000]
            uint16_t    Rear_wheel_speed;           // Rear wheel speed, [km/h],  [0km/h - 64km/h], [(256*A+B)/1024]
//            uint16_t    Rear_wheel_speed_avg;       // Rear wheel speed, [km/h],  [0km/h - 65.535km/h], [(256*A+B)/1000]
            uint16_t    Motor_RPM;                  // Inside hub motor estimate rpm, [rpm],   [0rpm - rpm], [(256*A+B)/]
            uint16_t    Cadence_RPM;                // Cyclist cadence, [rpm], [0rpm - 255rpm], [(256*A+B)/256]
//            uint16_t    Cadence_RPM_avg;            // Cyclist cadence, [rpm], [0rpm - 218.45rpm], [(256*A+B)/300]
//            uint16_t    Cadence_RPM_max;            // Cyclist cadence, [rpm], [0rpm - 218.45rpm], [(256*A+B)/300]

            // Reserved: uint32_t    PIDs_21__40;
            // Calculated variables
            uint16_t    Battery_power;              // Power out of the batteries, [W], [0W - 2048W], [(256*A+B)/32]
//            uint16_t    Battery_power_max;        // Max power out of the batteries, [W], [0W - 1310.7W], [(256*A+B)/50]
            uint16_t    Battery_capacity_used;      // Current consumed in Amp-hours, [mAh], [0Ah - 16,384mAh], [(256*A+B)/4]
            uint16_t    Energy_used;                // Energy consumed in kiloWatt-hours, [kWh], [0kWh - 1kWh], [(256*A+B)/65536]
            uint16_t    Battery_SOC;                // Battery State of Charge, [%], [0% - 128%], [(256*A+B)/512]
            uint16_t    Battery_resistance;         // Battery equivalent series resistance, [mOhm],  [0mOhm - 1024 mOhm], [(256*A+B)/64]
            uint16_t    Trip_distance;              // Trip distance based on rear wheel, [km], [0 - 256km], [(256*A+B)]
//            uint16_t    Trip_Time;                  // Trip Time in seconds, [s],  [0s - 65,535s], [(256*A+B)]
            uint16_t    Energy_mileage;             // Energy mileage, [Wh/km], [0Wh/km - 64Wh/km], [(256*A+B)/1024]
            uint8_t     Status;                     // Monitor Status, [Codes TBD], [0 CLEAR / >0 ERROR]

            // Reserved: uint32_t    PIDs_41__60;
            // Memory variables
//            uint16_t  Battery_cycles;               // Battery full cycles,  [p.u.] [0 - 1092.25]
//            uint16_t  Odometer;                     // Odometer, [km]  [0 - 65535km]

            // Reserved: uint32_t    PIDs_61__80;

        }Vars_t;


        // **************************************************************************
        // the globals

        extern Vars_t gVariables;


        // **************************************************************************
        // the functions prototypes

        //! \brief      processPid
        //! \param[in]  pid
        //! \param[in]  responseMsg
        void processPid(const std::string &pid, std::string &responseMsg);


    }; // mode21
}; //ELM327


#endif // !_MODE21_H_
