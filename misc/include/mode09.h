#ifndef _MODE09_H_
#define _MODE09_H_
//! \file   mode09.h
//! \brief  Request vehicle information
//! Not fully implemented. Just to avoid Torque errors
//! TODO: Implement multi line mode
//!

// **************************************************************************
// the includes

#include "types.h"
#include "utility.h"

#include <string>


// **************************************************************************
// the defines
#define MODE09_Vars_INIT { 0,    \
                           "",   \
                           "",   \
                           "",   }



namespace elm327 {

    namespace mode09 {

        // **************************************************************************
        // the typedefs

        //! \brief      Enumeration Mode 09 - Calibration_verification_numbers_.
        //! \details    As defined for ISO 9141-2, ISO 14230-4 and SAE J1850.
        typedef enum
        {
            PIDs_01__20 = 0x00,
        //    VIN_message_Count = 0x01,
            Vehicle_Identification_Number = 0x02,
        //    Calibration_ID_message_count = 0x03,
            Calibration_ID = 0x04,
        //    CVN_message_count = 0x05,
        //    Calibration_Verification_Numbers = 0x06,
        //    In_use_performance_tracking_message_count = 0x07,
        //    In_use_performance_tracking_for_spark_ignition_vehicles = 0x08,
        //    ECU_name_message_count = 0x09,
            ECU_name = 0x0A,
        //    In_use_performance_tracking_for_compression_ignition_vehicles = 0x0B

        } PIDs_e;

        typedef struct _Vars_t_
        {
            uint32_t    PIDs_01__20;
        //    uint8_t     VIN_message_Count;
            char        Vehicle_Identification_Number[17];
        //    uint8_t     Calibration_ID_message_count;
            char        Calibration_ID[16];
        //    uint8_t     CVN_message_count;
        //    uint32_t    Calibration_Verification_Numbers;
        //    uint8_t     In_use_performance_tracking_message_count;
        //    uint32_t    In_use_performance_tracking_for_spark_ignition_vehicles[4];
        //    uint8_t     ECU_name_message_count;
            char        ECU_name[20];
        //    uint32_t    In_use_performance_tracking_for_compression_ignition_vehicles[5];

        }Vars_t;

        typedef void (*MODE09_HANDLER)();


        // **************************************************************************
        // the globals

        extern Vars_t gVariables;


        // **************************************************************************
        // the functions prototypes

        void init();

        //! \brief      processPid
        //! \param[in]  pid
        //! \param[in]  responseMsg
        void processPid(const std::string &pid, std::string &responseMsg);

        //! \brief      pids01_20
        void pids01_20();
        void vehicleId();
        void calibrationId();
        void ecuName();

    }; // mode09
}; //ELM327

#endif // !_MODE09_H_
