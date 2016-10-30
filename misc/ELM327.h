
#ifndef _ELM327_H_
#define _ELM327_H_

#include <string>
//#include <stdio.h>
//#include <map>

//typedef std::string string;
using namespace std;

class ELM327
{
public:
	static ELM327& Instance();
//	ELM327(const ELM327&) = delete;
//	void operator=(const ELM327&) = delete;

	void ProcessCommand(const string &command, string &responseMsg);
	
	void DeviceDescription(string &responseMsg);
	void ReadVoltage(string &responseMsg);
	void Reset(string &responseMsg);
	void DescribeProtocolNumber(string &responseMsg);
	void EchoOffOn(string &responseMsg);
	void VersionId(string &responseMsg);
	void LinefeedsOffOn(string &responseMsg);
	void MemoryOffOn(string &responseMsg);


private:
	ELM327();

//    char* reset;

//	std::map<string, string(ELM327::*)(void)> m_functionCommands;

	float m_voltage;
	float m_version;

//	char* keyAt1;
//	char* keyZ;
};

#endif // !_ELM327_H_
