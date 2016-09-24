/* --COPYRIGHT--,BSD
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
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
#ifndef _MATH_H_
#define _MATH_H_

//! \file   modules/math/src/32b/math.h
//! \brief  Contains the public interface to the 
//!         math (MATH) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "IQmathLib.h"
#include "types.h"


//!
//!
//! \defgroup MATH MATH
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup MATH_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines conversion scale factor from N*m to lb*in
//!
#define MATH_Nm_TO_lbin_SF        (8.8507457913)

//! \brief Defines 4/3
//!
#define MATH_FOUR_OVER_THREE      (1.33333333333333333333333333333333)

//! \brief Defines 2/3
//!
#define MATH_TWO_OVER_THREE       (0.66666666666666666666666666666667)

//! \brief Defines 1/3
//!
#define MATH_ONE_OVER_THREE       (0.33333333333333333333333333333333)

//! \brief Defines 1/(pi)
//!
#define MATH_ONE_OVER_PI          (0.318309886183791)  

//! \brief Defines 1/sqrt(3)
//!
#define MATH_ONE_OVER_SQRT_THREE  (0.57735026918962576450914878050196)  

//! \brief Defines 1/(4*pi)
//!
#define MATH_ONE_OVER_FOUR_PI     (0.07957747154594767)  

//! \brief Defines 1/(2*pi)
//!
#define MATH_ONE_OVER_TWO_PI      (0.1591549430918954)  

//! \brief Defines pi
//!
#define	MATH_PI                   (3.1415926535897932384626433832795)

//! \brief Defines pi per unit
//!
#define	MATH_PI_PU                (0.5)

//! \brief Defines 2*pi
//!
#define	MATH_TWO_PI               (6.283185307179586)

//! \brief Defines 2*pi per unit 
//!
#define	MATH_TWO_PI_PU            (1.0)

//! \brief Defines 4*pi
//!
#define	MATH_FOUR_PI               (12.56637061435917)

//! \brief Defines 4*pi per unit
//!
#define	MATH_FOUR_PI_PU            (2.0)

//! \brief Defines pi/2
//!
#define	MATH_PI_OVER_TWO           (1.570796326794897)

//! \brief Defines pi/2 per unit
//!
#define	MATH_PI_OVER_TWO_PU        (0.25)

//! \brief Defines pi/4
//!
#define	MATH_PI_OVER_FOUR          (0.785398163397448)

//! \brief Defines pi/4 per unit
//!
#define	MATH_PI_OVER_FOUR_PU        (0.125)


#define rshft(A,n)	\
	(((n) < 2) ? rshft_1(A)	\
	:(((n) < 3) ? rshft_2(A)	\
	:(((n) < 4) ? rshft_3(A)	\
	:(((n) < 5) ? rshft_4(A)	\
	:(((n) < 6) ? rshft_5(A)	\
	:(((n) < 7) ? rshft_6(A)	\
	:(((n) < 8) ? rshft_7(A)	\
	:(((n) < 9) ? rshft_8(A)	\
	:(((n) < 10) ? rshft_9(A)	\
	:(((n) < 11) ? rshft_10(A)	\
	:(((n) < 12) ? rshft_11(A)	\
	:(((n) < 13) ? rshft_12(A)	\
	:(((n) < 14) ? rshft_13(A)	\
	:(((n) < 15) ? rshft_14(A)	\
	:(((n) < 16) ? rshft_15(A)	\
	:(((n) < 17) ? rshft_16(A)	\
	:(((n) < 18) ? rshft_17(A)	\
	:(((n) < 19) ? rshft_18(A)	\
	:(((n) < 20) ? rshft_19(A)	\
	:(((n) < 21) ? rshft_20(A)	\
	:(((n) < 22) ? rshft_21(A)	\
	:(((n) < 23) ? rshft_22(A)	\
	:(((n) < 24) ? rshft_23(A)	\
	:(((n) < 25) ? rshft_24(A)	\
	:(((n) < 26) ? rshft_25(A)	\
	:(((n) < 27) ? rshft_26(A)	\
	:(((n) < 28) ? rshft_27(A)	\
	:(((n) < 29) ? rshft_28(A)	\
	:(((n) < 30) ? rshft_29(A)	\
	:(((n) < 31) ? rshft_30(A)	\
	:(((n) < 32) ? rshft_31(A)	\
	:(rshft_32(A)))))))))))))))))))))))))))))))))

#define rshft_1(A)	(((A) + 0x1)>>1)
#define rshft_2(A)	(((A) + 0x2)>>2)
#define rshft_3(A)	(((A) + 0x4)>>3)
#define rshft_4(A)	(((A) + 0x8)>>4)
#define rshft_5(A)	(((A) + 0x10)>>5)
#define rshft_6(A)	(((A) + 0x20)>>6)
#define rshft_7(A)	(((A) + 0x40)>>7)
#define rshft_8(A)	(((A) + 0x80)>>8)
#define rshft_9(A)	(((A) + 0x100)>>9)
#define rshft_10(A)	(((A) + 0x200)>>10)
#define rshft_11(A)	(((A) + 0x400)>>11)
#define rshft_12(A)	(((A) + 0x800)>>12)
#define rshft_13(A)	(((A) + 0x1000)>>13)
#define rshft_14(A)	(((A) + 0x2000)>>14)
#define rshft_15(A)	(((A) + 0x4000)>>15)
#define rshft_16(A)	(((A) + 0x8000)>>16)
#define rshft_17(A)	(((A) + 0x10000)>>17)
#define rshft_18(A)	(((A) + 0x20000)>>18)
#define rshft_19(A)	(((A) + 0x40000)>>19)
#define rshft_20(A)	(((A) + 0x80000)>>20)
#define rshft_21(A)	(((A) + 0x100000)>>21)
#define rshft_22(A)	(((A) + 0x200000)>>22)
#define rshft_23(A)	(((A) + 0x400000)>>23)
#define rshft_24(A)	(((A) + 0x800000)>>24)
#define rshft_25(A)	(((A) + 0x1000000)>>25)
#define rshft_26(A)	(((A) + 0x2000000)>>26)
#define rshft_27(A)	(((A) + 0x4000000)>>27)
#define rshft_28(A)	(((A) + 0x8000000)>>28)
#define rshft_29(A)	(((A) + 0x10000000)>>29)
#define rshft_30(A)	(((A) + 0x20000000)>>30)
#define rshft_31(A)	(((A) + 0x40000000)>>31)
#define rshft_32(A)	(((A) + 0x80000000)>>32)

#define lshft(A,n)	((A)<<(n))

#define lshft_1(A)	((A)<<1)
#define lshft_2(A)	((A)<<2)
#define lshft_3(A)	((A)<<3)
#define lshft_4(A)	((A)<<4)
#define lshft_5(A)	((A)<<5)
#define lshft_6(A)	((A)<<6)
#define lshft_7(A)	((A)<<7)
#define lshft_8(A)	((A)<<8)
#define lshft_9(A)	((A)<<9)
#define lshft_10(A)	((A)<<10)
#define lshft_11(A)	((A)<<11)
#define lshft_12(A)	((A)<<12)
#define lshft_13(A)	((A)<<13)
#define lshft_14(A)	((A)<<14)
#define lshft_15(A)	((A)<<15)
#define lshft_16(A)	((A)<<16)
#define lshft_17(A)	((A)<<17)
#define lshft_18(A)	((A)<<18)
#define lshft_19(A)	((A)<<19)
#define lshft_20(A)	((A)<<20)
#define lshft_21(A)	((A)<<21)
#define lshft_22(A)	((A)<<22)
#define lshft_23(A)	((A)<<23)
#define lshft_24(A)	((A)<<24)
#define lshft_25(A)	((A)<<25)
#define lshft_26(A)	((A)<<26)
#define lshft_27(A)	((A)<<27)
#define lshft_28(A)	((A)<<28)
#define lshft_29(A)	((A)<<29)
#define lshft_30(A)	((A)<<30)
#define lshft_31(A)	((A)<<31)
#define lshft_32(A)	((A)<<32)


// **************************************************************************
// the typedefs

//! \brief Defines a two element vector
//!
typedef struct _MATH_vec2_
{

  _iq  value[2];

} MATH_vec2;


//! \brief Defines a three element vector
//!
typedef struct _MATH_vec3_
{

  _iq  value[3];

} MATH_vec3;


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _MATH_H_ definition


