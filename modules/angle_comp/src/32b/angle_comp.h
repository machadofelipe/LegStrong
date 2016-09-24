#ifndef _ANGLE_COMP_H_
#define _ANGLE_COMP_H_

//! \file   ~/sw/modules/angle_comp/src/float/angle_comp.h
//! \brief  Contains the public interface to the angle compensation
//! \brief  generator (ANGLE_COMP)
//!         module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <stdint.h>

// drivers 


// modules
#include "types.h"
#include "math.h"
#include "IQmathLib.h"

//!
//! \defgroup ANGLE_COMP

//!
//! \ingroup ANGLE_COMP
//@{


#ifdef __cplusplus
extern "C" {
#endif


//! \brief Defines the angle generator (ANGLE_COMP) object
//!
typedef struct _ANGLE_COMP_Obj_
{
  _iq            angleDeltaFactor;      //!< predetermined factor for use in angle compensation calculation
  _iq            angleCompFactor;       //!< predetermined factor for use in angle compensation calculation
  _iq            angleComp_pu;          //!< the angle compensation value
} ANGLE_COMP_Obj;


//! \brief Defines the ANGLE_COMP handle
//!
typedef struct _ANGLE_COMP_Obj_  *ANGLE_COMP_Handle;


// **************************************************************************
// the function prototypes


//! \brief     Gets the predicted angle value
//! \param[in] handle  The angle generator (ANGLE_COMP) handle
//! \return    The predicted angle compensation value, rad
static inline float_t ANGLE_COMP_getAngleComp_pu(ANGLE_COMP_Handle handle)
{
  ANGLE_COMP_Obj *obj = (ANGLE_COMP_Obj *)handle;

  return(obj->angleComp_pu);
} // end of ANGLE_COMP_getAngleComp_pu() function


//! \brief     Initializes the angle generator (ANGLE_COMP) module
//! \param[in] pMemory   A pointer to the memory for the object
//! \param[in] numBytes  The number of bytes allocated for the object, bytes
//! \return    The angle generator (ANGLE_COMP) object handle
extern ANGLE_COMP_Handle     ANGLE_COMP_init(void *pMemory,const size_t numBytes);


//! \brief  Compensates for the delay introduced
//! \brief  from the time when the system inputs are sampled to when the PWM
//! \brief  voltages are applied to the motor windings.
//! \param[in] handle     The angle generator (ANGLE_COMP) handle
//! \param[in] fm_pu      The electrical speed in pu
//! \param[in] angleUncomp_pu  The uncompensated angle in pu
static inline void ANGLE_COMP_run(ANGLE_COMP_Handle handle,const _iq fm_pu,const _iq angleUncomp_pu)
{
  ANGLE_COMP_Obj *obj = (ANGLE_COMP_Obj *)handle;
  _iq angleDelta_pu = _IQmpy(fm_pu,obj->angleDeltaFactor);
  _iq angleDeltaComp_pu = _IQmpy(angleDelta_pu,obj->angleCompFactor);

  uint32_t angleMask = ((uint32_t)0xFFFFFFFF >> (32 - GLOBAL_Q));
  _iq angleComp_pu;
  _iq angleTmp_pu;

  // increment the angle
  angleTmp_pu = angleUncomp_pu + angleDeltaComp_pu;

  // mask the angle for wrap around
  // note: must account for the sign of the angle
  angleComp_pu = _IQabs(angleTmp_pu) & angleMask;

  // account for sign
  if(angleTmp_pu < _IQ(0.0))
  {
      angleComp_pu = -angleComp_pu;
  }

  obj->angleComp_pu = angleComp_pu;

  return;
} // end of ANGLE_COMP_run()


//! \brief     Sets the parameters
//! \param[in] handle               The angle generator (ANGLE_COMP) handle
//! \param[in] iqFullScaleFreq_Hz   The frequency used to set 1 pu
//! \param[in] pwmPeriod_usec       The the pwmPeriod in usec
//! \param[in] numPwmTicksPerIsrTick  The decimation between PWM cycles and the ISR cycle
extern void ANGLE_COMP_setParams(ANGLE_COMP_Handle handle,
                            float_t iqFullScaleFreq_Hz,
                            float_t pwmPeriod_usec,
                            uint_least16_t numPwmTicksPerIsrTick);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _ANGLE_COMP_H_ definition

