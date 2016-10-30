
#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <string>
//#include <stdio.h>
//#include <string.h>

//typedef std::string string;
using namespace std;


#define HEADER_SIZE 2
//#define CMD_SIZE 5

class Message
{
public:
    Message(const string &pBuf);

    ~Message();

	string UnpackHeader(const string &pBuf);

	void HandleMessage(string &responseMsg);

private:

	// Message header
	string m_type;
	string m_command;

	//string responseMsg;

	// Message specific pack method, to be defined according to the message's
	// fields. The base class functions pack and unpack the common header
	int Pack(const string &pBuf);

	// Corresponding unpack method.
	int Unpack(const string &pBuf);


	// String index is used to keep track of the current packing/unpacking char index.
	int m_stringIndex;

	// PackString packs a specified number of chars at the next char index. The method increments
	// the char index
	void PackString(const string &pBuf, const string &value);

	// UnpackString packs a specified number of chars at the next char index.The method increments the char index
//	void UnpackString(const string &pBuf, const int &numChars);
	void UnpackCommand(const string &pBuf, const int &numBytes);
	void UnpackType(const string &pBuf);


};

#endif // ! _MESSAGE_H_
