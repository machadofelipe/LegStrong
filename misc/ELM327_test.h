#ifndef ELM327_TEST_H_
#define ELM327_TEST_H_
/*
 * ELM327.h
 *
 *  Created on: 06/11/2016
 *      Author: Felipe Machado
 */

//! \file   ELM327.h
//! \brief
//!


// **************************************************************************
// the includes

//#include <string.h>
#include <string>


// **************************************************************************
// the defines

#define COMMANDS_SIZE   8


// **************************************************************************
// the typedefs

//using namespace std;

typedef void (*HANDLER)(std::string &);

//! \brief Enumeration for response messages
//!
typedef enum
{
  ELM327_DeviceDescription=0,      //!< The DeviceDescription
  ELM327_VersionId,                //!< The VersionId
  ELM327_OK,                       //!< The OK message
  ELM327_ResponseMsg_size   //!< Size used for the array
} ELM327_ResponseMsg_e;


// **************************************************************************
// the globals

extern void ELM327_reset(std::string &responseMsg);
extern void ELM327_deviceDescription(std::string &responseMsg);
extern void ELM327_versionId(std::string &responseMsg);
extern void ELM327_readVoltage(std::string &responseMsg);
extern void ELM327_describeProtocolNumber(std::string &responseMsg);
extern void ELM327_memoryOffOn(std::string &responseMsg);
extern void ELM327_linefeedsOffOn(std::string &responseMsg);
extern void ELM327_echoOffOn(std::string &responseMsg);

//! \brief Array for Functions pointers
//!
static const HANDLER ELM327_Functions[COMMANDS_SIZE] = {
        &ELM327_reset,                     // Reset
        &ELM327_deviceDescription,         // DeviceDescription
        &ELM327_versionId,                 // VersionId
        &ELM327_readVoltage,               // ReadVoltage
        &ELM327_describeProtocolNumber,    // DescribeProtocolNumber
        &ELM327_memoryOffOn,               // MemoryOffOn
        &ELM327_linefeedsOffOn,            // LinefeedsOffOn
        &ELM327_echoOffOn,                 // EchoOffOn
};

//! \brief Array for Commands that points to each function pointer
//!
static const char* const ELM327_Commands[COMMANDS_SIZE] = {
        "Z",    // Reset
        "@1",   // DeviceDescription
        "I",    // VersionId
        "RV",   // ReadVoltage
        "DPN",  // DescribeProtocolNumber
        "M",    // MemoryOffOn
        "SP",   // LinefeedsOffOn
        "E",    // EchoOffOn
};

//! \brief Array for response messages
//!
static const char* const ELM327_ResponseMsg[ELM327_ResponseMsg_size] = {
        "Legstrong revA",   // DeviceDescription
        "ELM327 v2.1",      // VersionId
        "OK",               // OK
};

// **************************************************************************
// the function prototypes


//! \brief  ProcessCommand
//! \param[in] command
//! \param[in] responseMsg
void ELM327_processCommand(const std::string command, std::string &responseMsg);

//! \brief  Reset
//! \param[in] responseMsg
void ELM327_reset(std::string &responseMsg);

//! \brief  DeviceDescription
//! \param[in] responseMsg
void ELM327_deviceDescription(std::string &responseMsg);

//! \brief  VersionId
//! \param[in] responseMsg
void ELM327_versionId(std::string &responseMsg);

//! \brief  ReadVoltage
//! \param[in] responseMsg
void ELM327_readVoltage(std::string &responseMsg);

//! \brief  DescribeProtocolNumber
//! \param[in] responseMsg
void ELM327_describeProtocolNumber(std::string &responseMsg);

//! \brief  MemoryOffOn
//! \param[in] responseMsg
void ELM327_memoryOffOn(std::string &responseMsg);

//! \brief  LinefeedsOffOn
//! \param[in] responseMsg
void ELM327_linefeedsOffOn(std::string &responseMsg);

//! \brief  EchoOffOn
//! \param[in] responseMsg
void ELM327_echoOffOn(std::string &responseMsg);


#endif /* ELM327_TEST_H_ */
