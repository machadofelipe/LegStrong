#ifndef _MODE08_H_
#define _MODE08_H_
//! \file   mode08.h
//! \brief  Control operation of on-board component/system
//!

// **************************************************************************
// the includes

#include "types.h"
#include "utility.h"

#include <string>


// **************************************************************************
// the defines
//TODO: make one define for each
//TODO: in the future, store this values is memory
#define MODE08_Vars_INIT { false, 0x20, \
                           6, 2095, 2, false, 35, 15, 350, 45, }

namespace elm327 {


    // **************************************************************************
    // the class

    namespace mode08 {

    // **************************************************************************
    // the typedefs

    //! \brief      Enumeration Mode 08 - Control operation of on-board component/system
    //! \details    Codes defined specifically for this application
    typedef enum
    {
//        PIDs_01__20 = 0x00,
        Switch_Run = 0x01,
        Toggle_option = 0x02,

//        PIDs_21__40 = 0x20,
        Options_START = 0x20,
            PAS_magnets,
            Wheel_magnets,
            Wheel_circ,
            Throttle_ramp,
            Throttle_mode,
            Battery_cutout,
            Battery_limit,
            Power_limit,
            Speed_limit,
        Options_END,

//        PIDs_41__60 = 0x40,
        Increase_value = 0x41,
        Decresase_value = 0x42,
        Reset_all = 0x43,

//        PIDs_61__80 = 0x60,

    }PIDs_e;


    typedef struct _Vars_t_
    {
        // Reserved: uint32_t    PIDs_01__20;
        bool        Switch_Run;
        uint8_t     Toggle_option;

        // Reserved: uint32_t    PIDs_21__40;
        uint8_t     PAS_magnets;        // PAS magnets [magnets per turn]  [1 - 12]
//        uint8_t     Wheel_magnets;        // Wheel magnets [magnets per turn]  [1 - 12]
        uint16_t    Wheel_circ;         // Wheel Circumference [mm]
        uint8_t     Throttle_ramp;      // Throttle ramp [Slow/Med/Fast]  [1 - 3]
        bool        Throttle_mode;      // Throttle mode [torque controller / speed controller]  [false/true]
        uint8_t     Battery_cutout;     // Battery voltage cutout [V]  [0V - 60V]
        uint8_t     Battery_limit;      // Battery current limit  [A]  [0A - 20A]
        uint16_t    Power_limit;        // Power Limit  [W]
        uint8_t     Speed_limit;        // Rear wheel speed limit [km/h]

        // Reserved: uint32_t    PIDs_41__60;
        // Reserved: uint32_t    PIDs_61__80;

    }Vars_t;

    typedef void (*MODE08_HANDLER)();


    // **************************************************************************
    // the globals

    extern Vars_t gVariables;


    // **************************************************************************
    // the functions prototypes

    //! \brief      processPid
    //! \param[in]  pid
    //! \param[in]  responseMsg
    void processPid(const std::string &pid, std::string &responseMsg);

    //! \brief      PIDs_01__20
    // Reserved: void PIDs_01__20();
    void swRun();
    void togOption();

    //! \brief      pids21_40
    // Reserved: void pids21_40();
    void pasMagnets();
    void wheelCirc();
    void throttleRamp();
    void throttleMode();
    void batteryCutout();
    void batteryLimit();
    void powerLimit();
    void speedLimit();

    //! \brief      pids41_60
    // Reserved: void pids41_60();
    void increaseValue();
    void decreaseValue();
    void resetAll();

    //! \brief      pids61_80
    // Reserved: void pids61_80();


    }; // mode08
}; //ELM327


#endif // !_MODE08_H_
