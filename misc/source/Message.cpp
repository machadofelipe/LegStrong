//! \file   Message.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "Message.h"
#include "ELM327.h"
#include "mode01.h"
#include "mode08.h"
#include "mode09.h"
#include "mode21.h"


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the methods

Message::Message(const std::string &pBuf)
{
    UnpackMsg(pBuf);
}

Message::~Message() { }

void Message::UnpackMsg(const std::string &pBuf)
{

    if (pBuf.size() > HEADER_SIZE)
    {
        m_type = pBuf.substr(0, HEADER_SIZE);
        m_command = pBuf.substr(HEADER_SIZE);
    }

    return;
}

//TODO: Problems to sum 40h to the mode,
// results in bad value in Torque when != "41"
void Message::HandleMessage(std::string &responseMsg)
{
    responseMsg = "?";

    if (m_type == "01")
    {
        responseMsg = "41";
        elm327::mode01::processPid(m_command, responseMsg);
    }
    else if (m_type == "08")
    {
        responseMsg = "41";
        elm327::mode08::processPid(m_command, responseMsg);
    }
    else if (m_type == "09")
    {
        responseMsg = "41";
        elm327::mode09::processPid(m_command, responseMsg);
    }
    else if (m_type == "21")
    {
        responseMsg = "61";
        elm327::mode21::processPid(m_command, responseMsg);
    }
	else if (m_type == "AT")
	{
		ELM327_processCommand(m_command, responseMsg);
	}

    // DEBUG
//    puts((m_type+m_command+"\t"+responseMsg).c_str());

	return;
}
