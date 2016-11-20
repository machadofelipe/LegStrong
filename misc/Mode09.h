#ifndef _MODE09_H_
#define _MODE09_H_
//! \file   Mode09.h
//! \brief  Show current data
//!

// **************************************************************************
// the includes

#include <string>
#include "types.h"     // Device Headerfile and Examples Include File


// **************************************************************************
// the defines
#define MOD09_Vars_INIT { 0,                \
                          "",   \
                          "",           \
                          "",   }


// **************************************************************************
// the typedefs

typedef void (*MOD09_HANDLER)();

//! \brief      Enumeration Mode 09 - Calibration_verification_numbers_.
//! \details    As defined for ISO 9141-2, ISO 14230-4 and SAE J1850.
typedef enum
{
    Mode09_PIDs_01__20 = 0x00,
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

} Mode09_PIDs_e;

typedef struct _Mode09_Vars_t_
{
    uint32_t    Mode09_PIDs_01__20;
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

}Mode09_Vars_t;


// **************************************************************************
// the globals
extern void MOD09_PIDs_01__20();
extern void MOD09_Vehicle_Identification_Number();
extern void MOD09_Calibration_ID();
extern void MOD09_ECU_name();


//! \brief Array for Array for string commands that points to each function pointer
//!
static char MOD09_hexToASCII[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                     '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

// **************************************************************************
// the function prototypes

void MOD09_init();

//! \brief      MOD09_processPid
//! \param[in]  pid
//! \param[in]  responseMsg
void MOD09_processPid(const std::string &pid, std::string &responseMsg);

//! \brief      PIDs_01__20
void MOD09_PIDs_01__20();
void MOD09_Vehicle_Identification_Number();
void MOD09_Calibration_ID();
void MOD09_ECU_name();


#endif // !_MODE09_H_
