//! \file   ELM327.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "ELM327.h"
#include <string.h>
//#include <stdio.h>


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void ELM327_processCommand(const std::string &commandAndValue, std::string &responseMsg)
{

    bool flagContinue = true;
    for (int i=0; (flagContinue && i < ELM327_Commands_size); i++)
    {
        int commandSize = strlen(ELM327_Pairs[i].commandString);

        if ( !strcmp( commandAndValue.substr(0, commandSize).c_str(),
                        ELM327_Pairs[i].commandString ) )
        {
            ELM327_Pairs[i].functionPtr(responseMsg);
            flagContinue = false;
        }
    }

    return;
}

void ELM327_reset(std::string &responseMsg)
{
    ELM327_versionId(responseMsg);

    return;
}

void ELM327_deviceDescription(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_DeviceDescription];

    return;
}

void ELM327_versionId(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_VersionId];

    return;
}

void ELM327_readVoltage(std::string &responseMsg)
{
//    float vdcBus = 51.8;
//    char buf[5] = "";
//    snprintf( buf, 5, "%d.%d", (int)vdcBus, (int)( ( vdcBus - (int)vdcBus ) * 10 ) );

    responseMsg = "51.8";

    return;
}

void ELM327_describeProtocolNumber(std::string &responseMsg)
{
//    int protocolNumber = 1;
//    char buf[2] = "";
//    snprintf(buf, 2, "%d", protocolNumber);

    responseMsg = "1";

    return;
}

void ELM327_memoryOffOn(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}

void ELM327_linefeedsOffOn(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}

void ELM327_echoOffOn(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}


void ELM327_setTimeout(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}


void ELM327_spacesOffOn(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}


void ELM327_headersOffOn(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}


void ELM327_adaptiveTiming(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}


void ELM327_setProtocol(std::string &responseMsg)
{
    responseMsg = ELM327_ResponseMsg[ELM327_Msg_OK];

    return;
}
