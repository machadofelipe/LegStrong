/*
// TI File $Revision: /main/3 $
// Checkin $Date: March 16, 2012   14:54:07 $
//###########################################################################
//
// FILE:	F28052F.cmd
//
// TITLE:	Linker Command File For F28052F Device
//
//###########################################################################
// $TI Release: 2805x C/C++ Header Files vBeta1 $
// $Release Date: December 9, 2011 $
//###########################################################################
*/

/* ======================================================
// For Code Composer Studio V2.2 and later
// ---------------------------------------
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within
// the memory map.
//
// The header linker files are found in <base>\F28052F_Headers\cmd
//
// For BIOS applications add:      F28052F_Headers_BIOS.cmd
// For nonBIOS applications add:   F28052F_Headers_nonBIOS.cmd
========================================================= */

/* ======================================================
// For Code Composer Studio prior to V2.2
// --------------------------------------
// 1) Use one of the following -l statements to include the
// header linker command file in the project. The header linker
// file is required to link the peripheral structures to the proper
// locations within the memory map                                    */

/* Uncomment this line to include file only for non-BIOS applications */
/* -l F28052F_Headers_nonBIOS.cmd */

/* Uncomment this line to include file only for BIOS applications */
/* -l F28052F_Headers_BIOS.cmd */

/* 2) In your project add the path to <base>\F28052F_headers\cmd to the
   library search path under project->build options, linker tab,
   library search path (-i).
/*========================================================= */

/* Define the memory block start/length for the F28035
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F28052F are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks or flash sectors can be
         be combined if required to create a larger memory block.
*/

MEMORY
{
PAGE 0:    /* Program Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE1 for data allocation */
   RAML1L2			: origin = 0x008800, length = 0x000800		/* on-chip RAM block L1 */
/*   FLASHE			: origin = 0x3F0000, length = 0x002000		/* on-chip FLASH */
/*   FLASHD			: origin = 0x3F2000, length = 0x002000		/* on-chip FLASH */
   FLASHC			: origin = 0x3F0000, length = 0x006000		/* on-chip FLASH */
   FLASHA			: origin = 0x3F7000, length = 0x000FFE		/* on-chip FLASH */
   BEGIN			: origin = 0x3F7FFE, length = 0x000002		/* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   
   Z1_SCC_ROM		: origin = 0x3F8000, length = 0x000400		/* Zone 1 Safe-Copy Code Secure ROM */
   Z2_SCC_ROM		: origin = 0x3F8400, length = 0x000400		/* Zone 2 Safe-Copy Code Secure ROM */ 
   Z1_SECURE_ROM	: origin = 0x3F8808, length = 0x0044F8		/* Z1 Secure ROM */
   
   IQTABLES   		: origin = 0x3FDB52, length = 0x000b50  /* IQ Math Tables in Boot ROM */
   IQTABLES2  		: origin = 0x3FE6A2, length = 0x00008C  /* IQ Math Tables in Boot ROM */
   IQTABLES3  		: origin = 0x3FE72E, length = 0x0000AA  /* IQ Math Tables in Boot ROM */


   DCSM_OTP_Z2_P0	: origin = 0x3D7800, length = 0x000004		/* Part of Z1 OTP.  LinkPointer/JTAG lock/ Boot Mode */
   DCSM_OTP_Z1_P0	: origin = 0x3D7A00, length = 0x000006		/* Part of Z2 OTP.  LinkPointer/JTAG lock */
   
   /* DCSM Z1 Zone Select Contents and Reserved Locations (!!Movable!!) */
   /* Z1_DCSM_RSVD must be programmed to all 0x0000 and must immediately follow Z1 Zone Select block */
   DCSM_ZSEL_Z1_P0	: origin = 0x3D7A10, length = 0x000010		/* Part of Z1 OTP.  Z1 password locations / Flash and RAM partitioning */
   Z1_DCSM_RSVD     : origin = 0x3D7A20, length = 0x0001E0	    /* Part of Z1 OTP.  Program with all 0x0000 when Z1 DCSM is in use. */
   
   /* DCSM Z1 Zone Select Contents and Reserved Locations (!!Movable!!) */
   /* Z2_DCSM_RSVD must be programmed to all 0x0000 and must immediately follow Z2 Zone Select block */
   DCSM_ZSEL_Z2_P0	: origin = 0x3D7810, length = 0x000010		/* Part of Z2 OTP.  Z2 password locations / Flash and RAM partitioning  */
   Z2_DCSM_RSVD     : origin = 0x3D7820, length = 0x0001E0		/* Program with all 0x0000 when Z2 DCSM is in use. */
   
   ROM				: origin = 0x3FF27C, length = 0x000D44		/* Boot ROM */
   RESET			: origin = 0x3FFFC0, length = 0x000002		/* part of boot ROM  */
   VECTORS			: origin = 0x3FFFC2, length = 0x00003E		/* part of boot ROM  */

PAGE 1 :   /* Data Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE0 for program allocation */
           /* Registers remain on PAGE1                                                  */
   BOOT_RSVD		: origin = 0x000000, length = 0x000050		/* Part of M0, BOOT rom will use this for stack */
   RAMM0			: origin = 0x000050, length = 0x0003B0		/* on-chip RAM block M0 */
   RAMM1			: origin = 0x000400, length = 0x000400		/* on-chip RAM block M1 */
   RAML3			: origin = 0x009000, length = 0x001000		/* on-chip RAM block L3 */
   FLASHB			: origin = 0x3F6000, length = 0x001000		/* on-chip FLASH */

}

/* Allocate sections to memory blocks.
   Note:
         codestart user defined section in DSP28_CodeStartBranch.asm used to redirect code
                   execution when booting to flash
         ramfuncs  user defined section to store functions that will be copied from Flash into RAM
*/

SECTIONS
{

   /* Allocate program areas: */
   .cinit			: > FLASHC				PAGE = 0
   .pinit			: > FLASHC,				PAGE = 0
   .text			: > FLASHC				PAGE = 0
   codestart		: > BEGIN				PAGE = 0

   ramfuncs            : LOAD = FLASHA,
                         RUN = RAML1L2,
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         PAGE = 0
   
   dcsm_otp_z1		: > DCSM_OTP_Z1_P0		PAGE = 0
   dcsm_otp_z2		: > DCSM_OTP_Z2_P0		PAGE = 0
   
   dcsm_zsel_z1		: > DCSM_ZSEL_Z1_P0		PAGE = 0
   dcsm_rsvd_z1		: > Z1_DCSM_RSVD		PAGE = 0
   dcsm_zsel_z2		: > DCSM_ZSEL_Z2_P0		PAGE = 0
   dcsm_rsvd_z2		: > Z2_DCSM_RSVD		PAGE = 0

   /* Allocate uninitalized data sections: */
   .stack			: > RAMM0				PAGE = 1
   .ebss			: > RAML3				PAGE = 1
   .esysmem			: > RAML3				PAGE = 1

   /* Initalized sections go in Flash */
   /* For SDFlash to program these, they must be allocated to page 0 */
   .econst			: > FLASHC				PAGE = 0
   .switch			: > FLASHC				PAGE = 0

   /* Allocate IQ math areas: */
   IQmath			: > FLASHC				PAGE = 0            /* Math Code */
   IQmathTables		: > IQTABLES,			PAGE = 0, TYPE = NOLOAD

  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables2	: > IQTABLES2,			PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }
   */
    /* Uncomment the section below if calling the IQNasin() or IQasin()
       functions from the IQMath.lib library in order to utilize the
       relevant IQ Math table in Boot ROM (This saves space and Boot ROM
       is 1 wait-state). If this section is not uncommented, IQmathTables2
       will be loaded into other memory (SARAM, Flash, etc.) and will take
       up space, but 0 wait-state is possible.
    */
    /*
    IQmathTables3	: > IQTABLES3,			PAGE = 0, TYPE = NOLOAD
    {

               IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

    }
    */

   /* .reset is a standard section used by the compiler.  It contains the */
   /* the address of the start of _c_int00 for C Code.   /*
   /* When using the boot ROM this section and the CPU vector */
   /* table is not needed.  Thus the default type is set here to  */
   /* DSECT  */
   .reset			: > RESET,				PAGE = 0, TYPE = DSECT
   vectors			: > VECTORS				PAGE = 0, TYPE = DSECT

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/

