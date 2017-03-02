//###########################################################################
//
// FILE:    F2802x_BootVars.h
//          
// TITLE:   F2802x Boot Variable Definitions.
//
// NOTES:
//
//###########################################################################
// $TI Release: F2802x Support Library v230 $
// $Release Date: Fri May  8 07:43:05 CDT 2015 $
// $Copyright: Copyright (C) 2008-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef F2802x_BOOT_VARS_H
#define F2802x_BOOT_VARS_H

#ifdef __cplusplus
extern "C" {
#endif



//---------------------------------------------------------------------------
// External Boot ROM variable definitions:
//
extern uint16_t EmuKey;
extern uint16_t EmuBMode;
extern uint32_t Flash_CPUScaleFactor;
extern void (*Flash_CallbackPtr) (void);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of F2802x_BOOT_VARS_H definition

//===========================================================================
// End of file.
//===========================================================================

