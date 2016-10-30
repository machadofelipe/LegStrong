
#include "Message.h"
//#include "Mode01.h"
#include "ELM327.h"

//#include <stdlib.h>     /* malloc, free, rand */

Message::Message(const string &pBuf)
{
    Unpack(pBuf);
}

Message::~Message()
{
//    free(m_command);
}

int Message::Unpack(const string &pBuf)
{
	m_stringIndex = 0;
	UnpackType(pBuf);

	int charsToEnd = pBuf.size() - m_stringIndex;
	UnpackCommand(pBuf, charsToEnd);

	return m_stringIndex;
}

void Message::UnpackType(const string &pBuf)
{
//    if (strlen(pBuf) >= (HEADER_SIZE + m_stringIndex))
//    {
//        int j = 0;
//        for (int i = m_stringIndex; i < HEADER_SIZE; i++)
//        {
//            m_type[j] = pBuf[i];
//            j++;
//        }
//        m_type[j] = '\0';   /* null character manually added */
//        m_stringIndex += HEADER_SIZE;
//    }
    if (pBuf.size() >= (HEADER_SIZE + m_stringIndex))
    {
        m_type = pBuf.substr(m_stringIndex, HEADER_SIZE);
        m_stringIndex += HEADER_SIZE;
    }


    return;
}


void Message::UnpackCommand(const string &pBuf, const int &numBytes)
{
//    m_command = (char*) malloc(numBytes+1);
//
//    if (strlen(pBuf) >= (numBytes + m_stringIndex))
//    {
//        int j = 0;
//        for (int i = m_stringIndex;
//                i < numBytes + m_stringIndex && j < CMD_SIZE;
//                i++)
//        {
//            m_command[j] = pBuf[i];
//            j++;
//        }
//        m_command[j] = '\0';   /* null character manually added */
//        m_stringIndex += numBytes;
//    }
//    else
//    {
//        sprintf(m_command, "");
//    }
    if (pBuf.size() >= (numBytes + m_stringIndex))
    {
        m_command = pBuf.substr(m_stringIndex, numBytes);
        m_stringIndex += numBytes;
    }

    return;
}

//string Message::UnpackString(const string &pBuf, const int &numBytes)
//{
//	//TODO: maybe better to put as a member
//	 result = (char*) malloc(numBytes+1);
//	result = "";
//
//    if (strlen(pBuf) >= (numBytes + m_stringIndex))
//	{
//		//result = pBuf.substr(m_stringIndex, numBytes);
//		int j = 0;
//		for (int i = m_stringIndex; i < numBytes; i++)
//		{
//			result[j] = pBuf[i];
//			j++;
//		}
//		result[j] = '\0';   /* null character manually added */
//		m_stringIndex += numBytes;
//	}
//
//	return result;
//}

void Message::HandleMessage(string &responseMsg)
{
	//TODO: maybe better to put as a member
//	string responseMsg = (char*) malloc(40);

    if (m_type == "01")
	{
//		Mode01::Instance().ProcessPid(m_command, responseMsg);
	}
	else if (m_type == "AT")
	{
		ELM327::Instance().ProcessCommand(m_command, responseMsg);
	}
	else
	{
	    responseMsg = "?";
	}

	return;
}
