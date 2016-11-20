//! \file   Message.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "Message.h"
#include "ELM327.h"
#include "Mode01.h"
#include "Mode09.h"


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

void Message::HandleMessage(std::string &responseMsg, Mode01_Vars_t &gMod01Vars)
{
    responseMsg = "?";

    if (m_type == "01")
    {
        responseMsg = "41";
        MOD01_processPid(m_command, responseMsg, gMod01Vars);
    }
    if (m_type == "09")
    {
        responseMsg = "41";
        MOD09_processPid(m_command, responseMsg);
    }
	else if (m_type == "AT")
	{
		ELM327_processCommand(m_command, responseMsg);
	}

    // DEBUG
//    puts((m_type+m_command+"\t"+responseMsg).c_str());

	return;
}
