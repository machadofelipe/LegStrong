#ifndef __SPINTAC_VERSION_H__
#define __SPINTAC_VERSION_H__
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file    modules/spintac/src/32b/spintac_version.h
//! \brief   Public interface, object, and function definitions related to the
//!          SpinTAC Version object
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup SPINTACVER SpinTAC Version
//@{

//! \brief      Defines the math implementations available for the SpinTAC Library
typedef enum {
	FIXED_POINT_16b = 0,
	FIXED_POINT_32b,
	FLOAT_POINT_32b
} ST_MathType_e;

//! \brief      Defines the version data
//! \details    The ST_Ver_t object contains all parameters needed to
//!				perform the Version function.
typedef struct {
  uint16_t		Major;
  uint16_t		Minor;
  uint16_t		Revision;
  ST_MathType_e MathType;
  uint32_t		SecureROM;
  int32_t		Date;
  uint_least8_t	Label[10];
} ST_Ver_t;	// Structure for SpinTAC Version

typedef struct _ST_VER_Handle_ *ST_VER_Handle; // SpinTAC Version Handle

//! \brief      Gets the Version Number (Major, Minor, Revision) for SpinTAC Version
//! \param[in]  handle    The handle for the SpinTAC Version Object
//! \param[out] *major    Major version number
//! \param[out] *minor    Minor version number
//! \param[out] *revision Revision version number
static inline void ST_getVersionNumber(ST_VER_Handle handle, uint16_t *major, uint16_t *minor, uint16_t *revision) {
	ST_Ver_t *obj = (ST_Ver_t *)handle;

	*major = obj->Major;
	*minor = obj->Minor;
	*revision = obj->Revision;
} // end of ST_getVersionNumber function

//! \brief      Gets the Version Math Implementation (FixedPt) for SpinTAC Version
//! \param[in]  handle                 The handle for the SpinTAC Version Object
//! \return 	ST_MathType_e MathType Math implementation used in library
static inline int32_t ST_getVersionMath(ST_VER_Handle handle) {
	ST_Ver_t *obj = (ST_Ver_t *)handle;

	return(obj->MathType);
} // end of ST_getVersionMath function

//! \brief      Gets the Version Date (Date) for SpinTAC Version
//! \param[in]  handle       The handle for the SpinTAC Version Object
//! \return 	int32_t Date Date the library was compiled { format: YYYYMMDD }
static inline int32_t ST_getVersionDate(ST_VER_Handle handle) {
	ST_Ver_t *obj = (ST_Ver_t *)handle;

	return(obj->Date);
} // end of ST_getVersionDate function

//! \brief      Gets the ROM Version Number (SecureROM) for SpinTAC Version
//! \param[in]  handle            The handle for the SpinTAC Version Object
//! \return 	int32_t SecureROM Secure ROM Version Number
static inline int32_t ST_getSecureROMVersion(ST_VER_Handle handle) {
	ST_Ver_t *obj = (ST_Ver_t *)handle;

	return(obj->SecureROM);
} // end of ST_getSecureROMVersion function

//! \brief      Initializes the SpinTAC Version object
//! \param[in]  *pMemory             Pointer to the memory for ST_Ver_t
//! \param[in]  numBytes             The number of bytes in the ST_Ver_t
//! \return   	ST_VER_Handle handle The handle for the SpinTAC Version Object
ST_VER_Handle ST_initVersion(void *pMemory, const size_t numBytes);

//@} // defgroup
#endif //__SPINTAC_VERSION_H__
