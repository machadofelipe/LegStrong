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

//TODO: Change BATTERY_CELLS to a mode 08 setting
//TODO: redo the battery cutout based on this number
#define BATTERY_CELLS         14.0


//TODO: make one define for each
//TODO: in the future, store this values is memory
#define MODE08_Vars_INIT { false, 0x20, \
                           12, 6, 2095, 2, /*false,*/ 35, 10, /*350, 45,*/ \
                           0 }

#define TORQUE_CTRL         0
#define SPEED_CTRL          1

#define WEAK_THROTTLE       1
#define MEDIUM_THROTTLE     2
#define STRONG_THROTTLE     3
#define BOOST_THROTTLE      4

#define BOOST_TIMER         59

#define MAX_PAS_MAGNETS     12
#define MIN_PAS_MAGNETS     1

#define MAX_WHEEL_MAGNETS   12
#define MIN_WHEEL_MAGNETS   1

#define MAX_WHEEL_CIRC      2326
#define MIN_WHEEL_CIRC      935
#define WHEEL_CIRC_STEPS    5

#define MAX_THROTTLE_RAMP   3
#define MIN_THROTTLE_RAMP   1

#define MAX_BAT_CUTOUT      60
#define MIN_BAT_CUTOUT      7

#define MAX_BAT_LIMIT      20
#define MIN_BAT_LIMIT      1


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
        uint8_t     Wheel_magnets;        // Wheel magnets [magnets per turn]  [1 - 12]
        uint16_t    Wheel_circ;         // Wheel Circumference [mm]
        uint8_t     Throttle_ramp;      // Throttle ramp [Weak/Med/Strong]  [1 - 3]
//        bool        Throttle_mode;      // Throttle mode [torque controller / speed controller]  [false/true]
        uint8_t     Battery_cutout;     // Battery voltage cutout [V]  [0V - 60V]
        //TODO: Battery_limit is being used as Max. motor current (IqRef_A)
        // This current is input for the throttle ramp
        uint8_t     Battery_limit;      // Battery current limit  [A]  [0A - 20A]
//        uint16_t    Power_limit;        // Power Limit  [W]
//        uint8_t     Speed_limit;        // Rear wheel speed limit [km/h]

        // Reserved: uint32_t    PIDs_41__60;
        // Reserved: uint32_t    PIDs_61__80;

        uint16_t    Boost_timer;

    }Vars_t;

    typedef void (*funcptr)();


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
    void swRun();
    void setBoost(bool boost);
    void togOption();

    //! \brief      pids21_40

    //! \brief      pids41_60
    void increaseValue();
    void decreaseValue();
    void resetAll();

    //! \brief      pids61_80


    }; // mode08
}; //ELM327


#endif // !_MODE08_H_
