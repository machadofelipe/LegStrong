
#include "ELM327.h"
//#include <set>
//#include <stdlib.h>     /* malloc, free, rand */


//#define KEY_LENGTH	5

//char* reset = "OBDII Emulator v1.0";


//inline string Command(std::map<string, string(ELM327::*)(void)>::const_iterator it) { return it->first; }
//inline string Function(std::map<string, string(ELM327::*)(void)>::const_iterator it, ELM327* elm327) { return (elm327->*it->second)(); }

ELM327::ELM327()
{
	m_voltage = 12.6;
	m_version = 1.0;

//	keyAt1 = (char*) malloc(KEY_LENGTH);
//	keyAt1 = "@1";
//	m_functionCommands[keyAt1] = &ELM327::DeviceDescription;
//
//	char *keyRV = (char*) malloc(KEY_LENGTH);
//	keyRV = "RV";
//	m_functionCommands[keyRV] = &ELM327::ReadVoltage;
//
//	keyZ = (char*) malloc(KEY_LENGTH);
//	keyRV = "Z";
//	m_functionCommands[keyZ] = &ELM327::Reset;
//
//	char *keyDPN = (char*) malloc(KEY_LENGTH);
//	keyDPN = "DPN";
//	m_functionCommands[keyDPN] = &ELM327::DescribeProtocolNumber;
//
//	char *keyE = (char*) malloc(KEY_LENGTH);
//	keyE = "E";
//	m_functionCommands[keyE] = &ELM327::EchoOffOn;
//
//	char *keyI = (char*) malloc(KEY_LENGTH);
//	keyI = "I";
//	m_functionCommands[keyI]	= &ELM327::VersionId;
//
//	char *keySP = (char*) malloc(KEY_LENGTH);
//	keySP = "SP";
//	m_functionCommands[keySP] = &ELM327::LinefeedsOffOn;
//
//	char *keyM = (char*) malloc(KEY_LENGTH);
//	keyM = "M";
//	m_functionCommands[keyM]	= &ELM327::MemoryOffOn;

}

//TODO: Destructor for ELM327, remember to free the mallocs

ELM327& ELM327::Instance()
{
	// This line only runs once, thus creating the only instance in existence
	static ELM327 instance;

	return instance;
}


void ELM327::ProcessCommand(const string &commandAndValue, string &responseMsg)
{

//    sprintf(responseMsg, "?");


	// The Message Unpack() is not able to break the commands from the values
	// The command can have between one and three chars 
	// This for will break in a set of strings based on the number of chars
//	std::set<string> possibleCommands;
//	string::iterator itString = commandAndValue.begin();
//	for (int i = 1;
//			(i < 4) && (itString != commandAndValue.end());
//			i++)
//	{
//		possibleCommands.insert(commandAndValue.substr(0, i));
//	}

	// Look for a command that matches one of the strings in the set
//	for (std::set<string>::iterator it = possibleCommands.begin();
////	for (std::set::iterator it = possibleCommands.begin();
//		it!= possibleCommands.end();
//		it++)
//	{
//		std::map<string, string(ELM327::*)(void)>::iterator itMap = m_functionCommands.find(*it);
//		std::map::iterator itMap = m_functionCommands.find(*it);

//		std::map<string, string(ELM327::*)(void)>::iterator itMap = m_functionCommands.find(commandAndValue);
//		if (itMap != m_functionCommands.end())
//		{
//			// Now that the command is figure out, check for a value
//		    Function(itMap, this);
//		    sprintf(responseMsg, "TEST");
//		}

    if (commandAndValue == "Z")
    {
        Reset(responseMsg);
    }


//	}


	return;
}

void ELM327::DeviceDescription(string &responseMsg)
{
    responseMsg = "OBDII Emulator v1.0";
	return;
}

void ELM327::ReadVoltage(string &responseMsg)
{
    responseMsg = "12.6";
	return;
}

void ELM327::Reset(string &responseMsg)
{
    responseMsg = "Reset";
	return;
}

void ELM327::DescribeProtocolNumber(string &responseMsg)
{
    responseMsg = "1";
	return;
}

void ELM327::EchoOffOn(string &responseMsg)
{
    responseMsg = "EchoOffOn";
	return;
}

void ELM327::VersionId(string &responseMsg)
{
    responseMsg = "VersionId";
	return;
}

void ELM327::LinefeedsOffOn(string &responseMsg)
{
    responseMsg = "OK";
	return;
}

void ELM327::MemoryOffOn(string &responseMsg)
{
    responseMsg = "MemoryOffOn";
	return;
}
