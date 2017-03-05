#ifndef ELM327_H_
#define ELM327_H_
//! \file   ELM327.h
//! \brief
//!

// **************************************************************************
// the includes

#include <string>


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

typedef void (*ELM327_HANDLER)(std::string &);

//! \brief      ELM327_Pairs_t
//! \details
//!
typedef struct _ELM327_Pair_t_
{
    const char* const       commandString;      //!< the command string

    const ELM327_HANDLER    functionPtr;        //!< the function pointer

} ELM327_Pair_t;

//! \brief Enumeration for ELM327 commands
//!
typedef enum
{
  ELM327_CMD_reset=0,                   //!< Reset
  ELM327_CMD_deviceDescription,         //!< DeviceDescription
  ELM327_CMD_versionId,                 //!< VersionId
  ELM327_CMD_readVoltage,               //!< ReadVoltage
  ELM327_CMD_describeProtocolNumber,    //!< DescribeProtocolNumber
  ELM327_CMD_memoryOffOn,               //!< MemoryOffOn
  ELM327_CMD_linefeedsOffOn,            //!< LinefeedsOffOn
  ELM327_CMD_echoOffOn,                 //!< EchoOffOn
  ELM327_CMD_setTimeout,                //!< SetTimeout
  ELM327_CMD_spacesOffOn,               //!< SpacesOffOn
  ELM327_CMD_headersOffOn,              //!< HeadersOffOn
  ELM327_CMD_adaptiveTiming,            //!< AdaptiveTiming
  ELM327_CMD_setProtocol,               //!< SetProtocol
  ELM327_Commands_size                  //!< Size used for the array
} ELM327_Commands_e;

//! \brief Enumeration for ELM327 response messages
//!
typedef enum
{
  ELM327_Msg_DeviceDescription=0,       //!< The DeviceDescription
  ELM327_Msg_VersionId,                 //!< The VersionId
  ELM327_Msg_OK,                        //!< The OK message
  ELM327_Msg_ResponseMsg_size           //!< Size used for the array
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
extern void ELM327_setTimeout(std::string &responseMsg);
extern void ELM327_spacesOffOn(std::string &responseMsg);
extern void ELM327_headersOffOn(std::string &responseMsg);
extern void ELM327_adaptiveTiming(std::string &responseMsg);
extern void ELM327_setProtocol(std::string &responseMsg);

//! \brief Array for Array for string commands that points to each function pointer
//!
static ELM327_Pair_t ELM327_Pairs[ELM327_Commands_size] = {
        {"Z",   &ELM327_reset},                     // Reset
        {"@1",  &ELM327_deviceDescription},         // DeviceDescription
        {"I",   &ELM327_versionId},                 // VersionId
        {"RV",  &ELM327_readVoltage},               // ReadVoltage
        {"DPN", &ELM327_describeProtocolNumber},    // DescribeProtocolNumber
        {"M",   &ELM327_memoryOffOn},               // MemoryOffOn
        {"L",   &ELM327_linefeedsOffOn},            // LinefeedsOffOn
        {"E",   &ELM327_echoOffOn},                 // EchoOffOn
        {"ST",  &ELM327_setTimeout},                // SetTimeout
        {"S",   &ELM327_spacesOffOn},               // SpacesOffOn
        {"H",   &ELM327_headersOffOn},              // HeadersOffOn //TODO
        {"AT",  &ELM327_adaptiveTiming},            // AdaptiveTiming
        {"SP",  &ELM327_setProtocol}                // SetProtocol
};

//! \brief Array for response messages
//!
static const char* const ELM327_ResponseMsg[ELM327_Msg_ResponseMsg_size] = {
        "Legstrong revA",   // DeviceDescription
        "ELM327 v1.0",      // VersionId
        "OK",               // OK
};

// **************************************************************************
// the function prototypes

//! \brief      ProcessCommand
//! \param[in]  command
//! \param[in]  responseMsg
void ELM327_processCommand(const std::string &command, std::string &responseMsg);

//! \brief      Reset
//! \param[in]  responseMsg
void ELM327_reset(std::string &responseMsg);

//! \brief      DeviceDescription
//! \param[in]  responseMsg
void ELM327_deviceDescription(std::string &responseMsg);

//! \brief      VersionId
//! \param[in]  responseMsg
void ELM327_versionId(std::string &responseMsg);

//! \brief      ReadVoltage
//! \param[in]  responseMsg
void ELM327_readVoltage(std::string &responseMsg);

//! \brief      DescribeProtocolNumber
//! \param[in]  responseMsg
void ELM327_describeProtocolNumber(std::string &responseMsg);

//! \brief      MemoryOffOn
//! \param[in]  responseMsg
void ELM327_memoryOffOn(std::string &responseMsg);

//! \brief      LinefeedsOffOn
//! \param[in]  responseMsg
void ELM327_linefeedsOffOn(std::string &responseMsg);

//! \brief      EchoOffOn
//! \param[in]  responseMsg
void ELM327_echoOffOn(std::string &responseMsg);

//! \brief      SetTimeout
//! \param[in]  responseMsg
void ELM327_setTimeout(std::string &responseMsg);

//! \brief      SpacesOffOn
//! \param[in]  responseMsg
void ELM327_spacesOffOn(std::string &responseMsg);

//! \brief      HeadersOffOn
//! \param[in]  responseMsg
void ELM327_headersOffOn(std::string &responseMsg);

//! \brief      AdaptiveTiming
//! \param[in]  responseMsg
void ELM327_adaptiveTiming(std::string &responseMsg);

//! \brief      SetProtocol
//! \param[in]  responseMsg
void ELM327_setProtocol(std::string &responseMsg);



// **************************************************************************
// AT Commands from version 1.0
// More in https://www.elmelectronics.com/wp-content/uploads/2016/07/AT_Command_Table.pdf
// Refer to document ELM327_AT_Commands.xlsx
//        {"AL",   &ELM327_}        // Allow Long (>7 byte) messages        (OBD)
//        {"BD",   &ELM327_}        // perform a Buffer Dump                (OBD)
//        {"BI",   &ELM327_}        // Bypass the Initialization sequence   (OBD)
//        {"CAF",  &ELM327_}        // CAN Automatic Formatting             (CAN)
//        {"CF",   &ELM327_}
//        {"CFC",  &ELM327_}
//        {"CM",   &ELM327_}
//        {"CP",   &ELM327_}
//        {"CS",   &ELM327_}
//        {"CV",   &ELM327_}
//        {"D",    &ELM327_}
//        {"DP",   &ELM327_}
//        {"IB",   &ELM327_}
//        {"MR",   &ELM327_}
//        {"MT",   &ELM327_}
//        {"NL",   &ELM327_}
//        {"PC",   &ELM327_}
//        {"R",    &ELM327_}
//        {"SH",   &ELM327_}
//        {"SW",   &ELM327_}
//        {"TP",   &ELM327_}
//        {"WM",   &ELM327_}
//        {"WS",   &ELM327_}

#endif /* ELM327_H_ */
