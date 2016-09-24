//###########################################################################
//
// FILE:	IQlog.h
//
// TITLE:	IQ log (natural) Math library functions definitions.
//
//###########################################################################
//
// Ver  | dd-mmm-yyyy |  Who  | Description of changes
// =====|=============|=======|==============================================
//  1.0 |  2 Aug 2010 | A. T. | Original Release.
// -----|-------------|-------|----------------------------------------------
//
//###########################################################################
// 

#ifndef __IQLOG_H_INCLUDED__
#define __IQLOG_H_INCLUDED__

//###########################################################################
#if MATH_TYPE == IQ_MATH
//###########################################################################
// If IQ_MATH is used, the following IQmath library function definitions
// are used:
//===========================================================================
extern    long _IQ30log(long A);
extern    long _IQ29log(long A);
extern    long _IQ28log(long A);
extern    long _IQ27log(long A);
extern    long _IQ26log(long A);
extern    long _IQ25log(long A);
extern    long _IQ24log(long A);
extern    long _IQ23log(long A);
extern    long _IQ22log(long A);
extern    long _IQ21log(long A);
extern    long _IQ20log(long A);
extern    long _IQ19log(long A);
extern    long _IQ18log(long A);
extern    long _IQ17log(long A);
extern    long _IQ16log(long A);
extern    long _IQ15log(long A);
extern    long _IQ14log(long A);
extern    long _IQ13log(long A);
extern    long _IQ12log(long A);
extern    long _IQ11log(long A);
extern    long _IQ10log(long A);
extern    long _IQ9log(long A);
extern    long _IQ8log(long A);
extern    long _IQ7log(long A);
extern    long _IQ6log(long A);
extern    long _IQ5log(long A);
extern    long _IQ4log(long A);
extern    long _IQ3log(long A);
extern    long _IQ2log(long A);
extern    long _IQ1log(long A);

#if GLOBAL_Q == 30
#define   _IQlog(A)  _IQ30log(A)
#endif
#if GLOBAL_Q == 29
#define   _IQlog(A)  _IQ29log(A)
#endif
#if GLOBAL_Q == 28
#define   _IQlog(A)  _IQ28log(A)
#endif
#if GLOBAL_Q == 27
#define   _IQlog(A)  _IQ27log(A)
#endif
#if GLOBAL_Q == 26
#define   _IQlog(A)  _IQ26log(A)
#endif
#if GLOBAL_Q == 25
#define   _IQlog(A)  _IQ25log(A)
#endif
#if GLOBAL_Q == 24
#define   _IQlog(A)  _IQ24log(A)
#endif
#if GLOBAL_Q == 23
#define   _IQlog(A)  _IQ23log(A)
#endif
#if GLOBAL_Q == 22
#define   _IQlog(A)  _IQ22log(A)
#endif
#if GLOBAL_Q == 21
#define   _IQlog(A)  _IQ21log(A)
#endif
#if GLOBAL_Q == 20
#define   _IQlog(A)  _IQ20log(A)
#endif
#if GLOBAL_Q == 19
#define   _IQlog(A)  _IQ19log(A)
#endif
#if GLOBAL_Q == 18
#define   _IQlog(A)  _IQ18log(A)
#endif
#if GLOBAL_Q == 17
#define   _IQlog(A)  _IQ17log(A)
#endif
#if GLOBAL_Q == 16
#define   _IQlog(A)  _IQ16log(A)
#endif
#if GLOBAL_Q == 15
#define   _IQlog(A)  _IQ15log(A)
#endif
#if GLOBAL_Q == 14
#define   _IQlog(A)  _IQ14log(A)
#endif
#if GLOBAL_Q == 13
#define   _IQlog(A)  _IQ13log(A)
#endif
#if GLOBAL_Q == 12
#define   _IQlog(A)  _IQ12log(A)
#endif
#if GLOBAL_Q == 11
#define   _IQlog(A)  _IQ11log(A)
#endif
#if GLOBAL_Q == 10
#define   _IQlog(A)  _IQ10log(A)
#endif
#if GLOBAL_Q == 9
#define   _IQlog(A)  _IQ9log(A)
#endif
#if GLOBAL_Q == 8
#define   _IQlog(A)  _IQ8log(A)
#endif
#if GLOBAL_Q == 7
#define   _IQlog(A)  _IQ7log(A)
#endif
#if GLOBAL_Q == 6
#define   _IQlog(A)  _IQ6log(A)
#endif
#if GLOBAL_Q == 5
#define   _IQlog(A)  _IQ5log(A)
#endif
#if GLOBAL_Q == 4
#define   _IQlog(A)  _IQ4log(A)
#endif
#if GLOBAL_Q == 3
#define   _IQlog(A)  _IQ3log(A)
#endif
#if GLOBAL_Q == 2
#define   _IQlog(A)  _IQ2log(A)
#endif
#if GLOBAL_Q == 1
#define   _IQlog(A)  _IQ1log(A)
#endif
//###########################################################################
#else   // MATH_TYPE == FLOAT_MATH
//###########################################################################
// If FLOAT_MATH is used, the IQmath library function are replaced by
// equivalent floating point operations:
//===========================================================================
#define   _IQlog(A)           log(A)
#define   _IQ30log(A)         log(A)
#define   _IQ29log(A)         log(A)
#define   _IQ28log(A)         log(A)
#define   _IQ27log(A)         log(A)
#define   _IQ26log(A)         log(A)
#define   _IQ25log(A)         log(A)
#define   _IQ24log(A)         log(A)
#define   _IQ23log(A)         log(A)
#define   _IQ22log(A)         log(A)
#define   _IQ21log(A)         log(A)
#define   _IQ20log(A)         log(A)
#define   _IQ19log(A)         log(A)
#define   _IQ18log(A)         log(A)
#define   _IQ17log(A)         log(A)
#define   _IQ16log(A)         log(A)
#define   _IQ15log(A)         log(A)
#define   _IQ14log(A)         log(A)
#define   _IQ13log(A)         log(A)
#define   _IQ12log(A)         log(A)
#define   _IQ11log(A)         log(A)
#define   _IQ10log(A)         log(A)
#define   _IQ9log(A)          log(A)
#define   _IQ8log(A)          log(A)
#define   _IQ7log(A)          log(A)
#define   _IQ6log(A)          log(A)
#define   _IQ5log(A)          log(A)
#define   _IQ4log(A)          log(A)
#define   _IQ3log(A)          log(A)
#define   _IQ2log(A)          log(A)
#define   _IQ1log(A)          log(A)
//###########################################################################
#endif  // No more.
//###########################################################################

#endif /* __IQLOG_H_INCLUDED__ */
