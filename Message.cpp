//! \file   Message.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "Message.h"
#include "ELM327.h"
#include "Mode09.h"
#include "mode08.h"


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

void Message::HandleMessage(std::string &responseMsg, elm327::mod01 &mod01)
{
    responseMsg = "?";

    if (m_type == "01")
    {
        responseMsg = "41";
        mod01.processPid(m_command, responseMsg);
    }
    if (m_type == "08")
    {
        responseMsg = "48";
        elm327::mode08::processPid(m_command, responseMsg);
    }
    if (m_type == "09")
    {
        responseMsg = "49";
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
