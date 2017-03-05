#ifndef MISC_UTILITYH_
#define MISC_UTILITYH_
//! \file   utility.h
//! \brief  Utility functions and defines
//!

// **************************************************************************
// the includes

#include <string>
#include "types.h"

namespace utility {

    // **************************************************************************
    // the defines

    typedef enum
    {
        UINT8 = 8,
        UINT16 = 16,
        UINT32 = 32,
        FN_PTR = 99,
    } Types_t;

    //! \brief Array for conversion hex -> char
    //!
    static const char hexToASCII[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                         '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };


    // **************************************************************************
    // the functions

    //! \brief      printHex
    //! \param[in]  string
    //! \param[in]  value
    void printHex(std::string &string, const uint32_t &value);

    //! \brief      printHex
    //! \param[in]  string
    //! \param[in]  value
    void printHex(std::string &string, const uint16_t &value);

    //! \brief      printHex
    //! \param[in]  string
    //! \param[in]  value
    void printHex(std::string &string, const uint8_t &value);

    //! \brief      printHex
    //! \param[in]  string
    //! \param[in]  value
    void printHex(std::string &string, const bool &value);

}
#endif /* MISC_UTILITYH_ */
