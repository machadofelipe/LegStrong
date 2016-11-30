//! \file   Mode01.cpp
//! \brief
//!

// **************************************************************************
// the includes

#include "utility.h"


// **************************************************************************
// the defines

using namespace utility;


// **************************************************************************
// the globals


void utility::printHex(std::string &string, const uint32_t &value)
{
    for (int i=7; i >= 0; i--)
    {
        string += (hexToASCII[((value >> 4*i) & 0x0000000F)]);
    }

    return;
}

void utility::printHex(std::string &string, const uint16_t &value)
{
    for (int i=3; i >= 0; i--)
    {
        string += (hexToASCII[((value >> 4*i) & 0x000F)]);
    }

    return;
}


void utility::printHex(std::string &string, const uint8_t &value)
{
    for (int i=1; i >= 0; i--)
    {
        string += (hexToASCII[((value >> 4*i) & 0x0F)]);
    }

    return;
}

