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
#include <map>


// **************************************************************************
// the defines
#define MODE08_Vars_INIT { false, 0 }

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
        // Reserved: Mode08_PIDs_01__20 = 0x00,
        SW_Run = 0x01,
        TOG_PowerMode = 0x02,

        // Reserved: Mode08_PIDs_21__40 = 0x20,

        // Reserved: Mode08_PIDs_41__60 = 0x40,

        // Reserved: Mode08_PIDs_61__80 = 0x60,
    }PIDs_e;


    typedef struct _Vars_t_
    {
        // Reserved: uint32_t    Mode08_PIDs_01__20;
        bool        SW_Run;
        uint8_t     TOG_PowerMode;
        // Reserved: uint32_t    Mode08_PIDs_21__40;

        // Reserved: uint32_t    Mode08_PIDs_41__60;

        // Reserved: uint32_t    Mode08_PIDs_61__80;

    }Vars_t;

    typedef void (*MODE08_HANDLER)();

//        public:

            //! \brief      processPid
            //! \param[in]  pid
            //! \param[in]  responseMsg
            void processPid(const std::string &pid, std::string &responseMsg);

            extern Vars_t gVariables;

            void swRunCall();

            void togPowerModeCall();


//        private:
//
//            //! \brief      PIDs_01__20
//            // Reserved: void PIDs_01__20();
//            void swRun();
//            void togPowerMode();
//
//            //! \brief      PIDs_21__40
//            // Reserved: void PIDs_21__40();
//
//            //! \brief      PIDs_41__60
//            // Reserved: void PIDs_41__60();
//
//            //! \brief      PIDs_61__80
//            // Reserved: void PIDs_61__80();
//
//
//            static std::map< int, MODE08_HANDLER > _pidMap;
//
//            static std::string _responseMsg;


    }; // mode08
}; //ELM327


#endif // !_MODE08_H_
