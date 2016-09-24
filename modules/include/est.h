#ifndef _EST_H_
#define _EST_H_

//! \file   modules/est/src/32b/est.h
//! \brief  Contains the public interface to the 
//!         estimator (EST) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "math.h"
#include "IQmathLib.h"
#include "est_states.h"


//!
//!
//! \defgroup EST EST
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup EST_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the Rs online filter types
//!
typedef enum
{
  EST_RsOnLineFilterType_Current=0,        //!< Current Filter
  EST_RsOnLineFilterType_Voltage           //!< Voltage Filter
} EST_RsOnLineFilterType_e;


//! \brief Defines the estimator (EST) handle
//!
typedef struct _EST_Obj_ *EST_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Computes the rotor inductance in Henries (H)
//! \param[in] handle   The estimator (EST) handle
//! \param[in] current  The current in the rotor 
//! \return    The rotor inductance, H
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_computeLr_H(EST_Handle handle,const _iq current);
#else
extern float_t EST_computeLr_H(EST_Handle handle,const _iq current);
#endif


//! \brief     Determines if current control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform current control
extern bool EST_doCurrentCtrl(EST_Handle handle);


//! \brief     Determines if speed control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform speed control
extern bool EST_doSpeedCtrl(EST_Handle handle);


//! \brief     Generated the PID Id controller output limits
//! \param[in] handle        The estimator (EST) handle
//! \param[in] maxDutyCycle  The maximum duty cycle, pu
//! \param[in] outMin        The pointer to the minimum output value
//! \param[in] outMax        The pointer to the maximum output value
extern void EST_genOutputLimits_Pid_Id(EST_Handle handle,
                                       const _iq maxDutyCycle,
                                       _iq *outMin,_iq *outMax);


//! \brief     Generated the PID Id controller output limits
//! \param[in] handle        The estimator (EST) handle
//! \param[in] maxDutyCycle  The maximum duty cycle, pu
//! \param[in] out_Id        The Id output value
//! \param[in] outMin        The pointer to the minimum output value
//! \param[in] outMax        The pointer to the maximum output value
extern void EST_genOutputLimits_Pid_Iq(EST_Handle handle,
                                       const _iq maxDutyCycle,
                                       const _iq out_Id,
                                       _iq *outMin,_iq *outMax);


//! \brief     Gets the krpm to pu scale factor in per unit (pu), IQ24.
//! \details   This function is needed when a user needs to scale a value of the motor speed from 
//!            kpm (kilo revolutions per minute) to a per units value. This scale factor is calculated
//!            and used as shown below:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS       (2)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz      (500.0)
//!
//! _iq scale_factor = _IQ(USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (60.0 * USER_IQ_FULL_SCALE_FREQ_Hz));
//!
//! _iq Speed_krpm = EST_getSpeed_krpm(handle);
//! _iq Speed_krpm_to_pu_sf = EST_get_krpm_to_pu_sf(handle);
//! _iq Speed_pu = _IQmpy(Speed_krpm,Speed_krpm_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The krpm to pu scale factor. This value is in IQ24.
extern _iq EST_get_krpm_to_pu_sf(EST_Handle handle);


//! \brief     Gets the pu to krpm scale factor in per unit (pu), IQ24.
//! \details   This function is needed when a user needs to scale a value of the motor speed from per 
//!            units to krpm (kilo revolutions per minute) value. This scale factor is calculated as follows:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS       (2)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz      (500.0)
//!
//! _iq scale_factor = IQ(60.0 * USER_IQ_FULL_SCALE_FREQ_Hz / (USER_MOTOR_NUM_POLE_PAIRS * 1000.0));
//!
//! _iq Speed_pu = EST_getFm_pu(handle);
//! _iq Speed_pu_to_krpm_sf = EST_get_pu_to_krpm_sf(handle);
//! _iq Speed_krpm = _IQmpy(Speed_krpm,Speed_krpm_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The pu to krpm scale factor. This value is in IQ24.
extern _iq EST_get_pu_to_krpm_sf(EST_Handle handle);


//! \brief     Gets the angle value from the estimator in per unit (pu), IQ24.
//! \details   This function returns a per units value of the rotor flux angle. This value wraps around 
//!            at 1.0, so the return value is between 0x00000000 or _IQ(0.0) to 0x00FFFFFF or _IQ(1.0). 
//!            An example of using this angle is shown:
//! \code
//! _iq Rotor_Flux_Angle_pu = EST_getAngle_pu(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle value, pu, in IQ24.
extern _iq EST_getAngle_pu(EST_Handle handle);


//! \brief     Gets the DC bus value from the estimator in per unit (pu), IQ24.
//! \details   This value is originally passed as a parameter when calling function EST_run(). 
//!            A similar function can be simply reading what has been read and scaled by the ADC converter
//!            on pAdcData->dcBus. This value is used by the libraries internally to calculate one over 
//!            dcbus, which is a value used to compensate the proportional gains of the current 
//!            controllers. The following example shows how to use this function to calculate a DC bus value
//!            in kilo volts:
//! \code
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//!
//! _iq Vbus_pu = EST_getDcBus_pu(handle);
//! _iq Vbus_pu_to_kV_sf = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V / 1000.0);
//! _iq Vbus_kV = _IQmpy(Vbus_pu,Vbus_pu_to_kV_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The DC bus value, pu
extern _iq EST_getDcBus_pu(EST_Handle handle);


//! \brief     Gets the error code from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The error code
extern EST_ErrorCode_e EST_getErrorCode(EST_Handle handle);


//! \brief     Gets the electrical frequency of the motor in Hertz (Hz).
//! \details   This frequency, in Hz, is the frequency of currents and voltages going into the motor. 
//!            In order to get the speed of the motor, it is better to use EST_getFm().
//! \param[in] handle  The estimator (EST) handle
//! \return    The electrical frequency, Hz
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFe(EST_Handle handle);
#else
extern float_t EST_getFe(EST_Handle handle);
#endif


//! \brief     Gets the electrical frequency of the motor in per unit (pu), IQ24.
//! \details   Similar to EST_getFe() function, this function returns the electrical frequency
//!            of the motor in per units. In order to convert the electrical frequency from 
//!            per units to Hz, the user needs to multiply the returned value by the following
//!            scale factor:
//! \code
//! _iq Full_Scale_Freq_Elec_Hz = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz);
//!
//! _iq Freq_Elec_Hz = _IQmpy(EST_getFe_pu(handle),Full_Scale_Freq_Elec_Hz);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The electrical frequency, pu
extern _iq EST_getFe_pu(EST_Handle handle);


//! \brief     Gets the enable force angle flag value from the estimator.
//! \param[in] handle  The estimator (EST) handle
//! \return    The value of the flag, in boolean type, bool
//! \retval    
//!            true  Forced angle is enabled, and the estimated angle will be bypassed if the flux
//!                  frequency falls below a threashold defined by:
//! \code 
//! #define USER_ZEROSPEEDLIMIT (0.001)
//! \endcode
//!                  A typical value of this frequency is 0.001 of the full scale frequency defined by:
//! \code 
//! #define USER_IQ_FULL_SCALE_FREQ_Hz (500.0) 
//! \endcode
//!                  The forced angle algorithm, when active, that is, when the rotor flux electrical
//!                  frequency falls below the threashold, will be forcing a rotating angle at a
//!                  frequency set by the following define:
//! \code 
//! #define USER_FORCE_ANGLE_FREQ_Hz  (1.0) 
//! \endcode
//! \retval
//!            false Force angle is disabled, and the estimator will never be bypassed by any forced
//!                  angle algorithm.
extern bool EST_getFlag_enableForceAngle(EST_Handle handle);


//! \brief     Gets the value of the flag which enables online stator resistance (Rs) estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable online Rs flag value
//! \retval
//!            true   Rs online recalibration algorithm is enabled. The estimator will run a set of
//!                   functions related to rs online which recalculates the stator resistance while the
//!                   motor is rotating. This algorithm is useful when motor heats up, and hence stator
//!                   resistance increases.
//! \retval
//!            false  Rs online recalibration algorithm is disabled, and no updates to Rs will be made
//!                   even if the motor heats up. Low speed performace, and startup performance with
//!                   full torque might be affected if stator resistance changes due to motor heating
//!                   up. The stator resistance will be fixed, and equal to the value returned by:
//!                   EST_getRs_Ohm().
extern bool EST_getFlag_enableRsOnLine(EST_Handle handle);


//! \brief     Gets the enable stator resistance re-calibrate flag value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The value of the enable stator resistance re-calibrate flag
//! \retval
//!            true   Rs recalibration is enabled. The estimator will inject a DC current to the D-axis
//!                   to recalibrate the stator resistance before the motor rotates. It is required that
//!                   the motor is at standstill to perform Rs recalibration. If online recalibration
//!                   of the stator resistance is needed, refer to EST_getFlag_enableRsOnLine() and
//!                   EST_setFlag_enableRsOnLine() functions.
//! \retval
//!            false  Rs recalibration is disabled. The estimator will start the motor with the resistance
//!                   value that was used before the motor was stopped, or what is returned by function:
//!                   EST_getRs_Ohm().
extern bool EST_getFlag_enableRsRecalc(EST_Handle handle);


//! \brief     Gets the value of the flag which denotes when the estimation is complete
//! \details   This flag is set to true every time the EST_run() function is run. 
//!            This flag can be reset to false by using the following example:
//! \code
//! bool estComplete_Flag = EST_getFlag_estComplete(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimation complete flag value
//! \retval
//!            true   The estimator has been run at least once since last time
//!                   EST_setFlag_estComplete(handle, false) was called.
//! \retval
//!            false  The estimator has not been run since last time EST_setFlag_estComplete(handle, false)
//!                   was called.
extern bool EST_getFlag_estComplete(EST_Handle handle);


//! \brief     Gets the value of the flag which enables the updating of the stator resistance (Rs) value
//! \details   When the online resistance estimator is enabled, the update flag allows the online resistance
//!            to be copied to the resistance used by the estimator model. If the update flag is not set to true,
//!            the online resistance estimation will not be used by the estimator model, and if the resistance
//!            changes too much due to temperature increase, the model may not work as expected.
//! \code
//! bool update_Flag = EST_getFlag_updateRs(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The update Rs flag value
//! \retval
//!            true   The stator resistance estimated by the Rs OnLine module will be copied to the'
//!                   the stator resistance used by the module, so of the motor's temperature changes,
//!                   the estimated angle will be calculated based on the most up to date stator
//!                   resistance
//! \retval
//!            false  The stator resistance estimated by the Rs OnLine module may or may not be updated
//!                   depending on the enable flag, but will not be used in the motor's model used to generate
//!                   the estimated speed and angle.
extern bool EST_getFlag_updateRs(EST_Handle handle);


//! \brief     Gets the flux value in Volts per Hertz (V/Hz).
//! \details   The estimator continuously calculates the flux linkage between the rotor and stator, which is the
//!            portion of the flux that produces torque. This function returns the flux linkage, ignoring the
//!            number of turns, between the rotor and stator coils, in Volts per Hertz, or V/Hz. This functions
//!            returns a precise value only after the motor has been identified, which can be checked by the
//!            following code example:
//! \code
//! if(EST_isMotorIdentified(handle))
//!   {
//!     // once the motor has been identified, get the flux
//!     float_t Flux_VpHz = EST_getFlux_VpHz(handle);
//!   }
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux value, V/Hz, in floating point
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFlux_VpHz(EST_Handle handle);
#else
extern float_t EST_getFlux_VpHz(EST_Handle handle);
#endif


//! \brief     Gets the flux value in Webers (Wb).
//! \details   The estimator continuously calculates the flux linkage between the rotor and stator, which is the
//!            portion of the flux that produces torque. This function returns the flux linkage, ignoring the
//!            number of turns, between the rotor and stator coils, in Webers, or Wb, or Volts * Seconds (V.s).
//!            This functions returns a precise value only after the motor has been identified, which can be 
//!            checked by the following code example:
//! \code
//! if(EST_isMotorIdentified(handle))
//!   {
//!     // once the motor has been identified, get the flux
//!     float_t Flux_Wb = EST_getFlux_Wb(handle);
//!   }
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux value, Webers or V.s, in floating point
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFlux_Wb(EST_Handle handle);
#else
extern float_t EST_getFlux_Wb(EST_Handle handle);
#endif


//! \brief     Gets the flux value in per unit (pu), IQ24.
//! \details   The estimator continuously calculates the flux linkage between the rotor and stator, which is the
//!            portion of the flux that produces torque. This function returns the flux linkage, ignoring the
//!            number of turns, between the rotor and stator coils, in per units.
//!            This functions returns a precise value only after the motor has been identified, which can be 
//!            checked by the following code example:
//! \code
//! if(EST_isMotorIdentified(handle))
//!   {
//!     // once the motor has been identified, get the flux
//!     _iq Flux_pu = EST_getFlux_pu(handle);
//!   }
//! \endcode
//! \details   For some applications it is important to get this value in per units, since it is much faster to
//!            process especially when the architecture of the microcontroller does not have a floating point
//!            processing unit. In order to translate this per units value into a scaled value in _iq, it is
//!            important to consider a scale factor to convert this flux in per units to the required units.
//!            The following example shows how to scale a per units value to Wb and V/Hz in IQ for faster
//!            processing:
//! \code
//! float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
//! float_t maxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
//! float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));
//! _iq gFlux_pu_to_Wb_sf = _IQ(FullScaleFlux/(2.0*MATH_PI)*pow(2.0,lShift));
//! _iq gFlux_pu_to_VpHz_sf = _IQ(FullScaleFlux*pow(2.0,lShift));
//! // The value of gFlux_pu_to_Wb_sf and gFlux_pu_to_VpHz_sf can be calculated once at the beginning of the 
//! // code and stored as global variables
//!
//! _iq Flux_Wb;
//! _iq Flux_VpHz;
//! _iq Flux_pu = EST_getFlux_pu(handle);
//!
//! Flux_Wb = _IQmpy(Flux_pu, gFlux_pu_to_Wb_sf);
//! Flux_VpHz = _IQmpy(Flux_pu, gFlux_pu_to_VpHz_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux value, pu
extern _iq EST_getFlux_pu(EST_Handle handle);


//! \brief     Gets the mechanical frequency of the motor in Hertz (Hz).
//! \details   This frequency, in Hz, is the mechanical frequency of the motor. If the motor is a permanent
//!            magnet motor, the mechanical frequency will be equal to the electrical frequency, since it is
//!            a synchronous motor. In the case of AC induction motors, the mechanical frequency will be equal
//!            to the electrical frequency minus the slip frequency. The following code example shows how to
//!            use this function to calculate revolutions per minute (RPM) in floating point:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS  (2)
//!
//! float_t Mechanical_Freq_Hz = EST_getFm(handle);
//! float_t hz_to_rpm_sf = 60.0/USER_MOTOR_NUM_POLE_PAIRS;
//! float_t Speed_RPM = Mechanical_Freq_Hz * hz_to_rpm_sf;
//! \endcode           
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, Hz
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFm(EST_Handle handle);
#else
extern float_t EST_getFm(EST_Handle handle);
#endif


//! \brief     Gets the mechanical frequency of the motor in per unit (pu), IQ24.
//! \details   Similar to EST_getFe_pu() function, this function returns the mechanical frequency
//!            of the motor in per units. In order to convert the mechanical frequency from 
//!            per units to kHz (to avoid saturation of IQ24), the user needs to multiply the 
//!            returned value by the following scale factor:
//! \code
//! #define USER_IQ_FULL_SCALE_FREQ_Hz  (500.0)
//!
//! _iq pu_to_khz_sf = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz/1000.0);
//! _iq khz_to_krpm_sf = _IQ(60.0/USER_MOTOR_NUM_POLE_PAIRS);
//! _iq Mechanical_Freq_kHz = _IQmpy(EST_getFm_pu(handle),pu_to_khz_sf);
//! _iq Speed_kRPM = _IQmpy(Mechanical_Freq_kHz,khz_to_krpm_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, pu
extern _iq EST_getFm_pu(EST_Handle handle);


//! \brief     Gets the force angle delta value from the estimator in per unit (pu), IQ24.
//! \details   This function returns a valid value only after initializing the controller object
//!            by calling CTRL_setParams() function. The force angle delta represents the increments
//!            to be added to or subtracted from the forced angle. The higher this value is, the higher
//!            frequency will be generated when the angle is forced (estimated angle is bypassed when 
//!            in forced angle mode). By default the forced angle frequency is set in user.h. 
//!            The following example shows how to convert delta in per units to kilo Hertz (kHz).
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_EST_TICK  (1)
//! #define USER_PWM_FREQ_kHz         (15.0)
//! #define USER_ISR_FREQ_Hz          (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz         (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_EST_FREQ_Hz          (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)
//!
//! _iq delta_pu_to_kHz_sf = _IQ((float_t)USER_EST_FREQ_Hz/1000.0);
//! _iq Force_Angle_Delta_pu = EST_getForceAngleDelta_pu(handle);
//! _iq Force_Angle_Freq_kHz = _IQmpy(Force_Angle_Delta_pu, delta_pu_to_kHz_sf);
//! \endcode
//! \note      Note that kHz is prefered to avoid overflow of IQ24 variables.
//! \param[in] handle  The estimator (EST) handle
//! \return    The force angle delta, pu. Minimum value of _IQ(0.0) and maximum of _IQ(1.0).
extern _iq EST_getForceAngleDelta_pu(EST_Handle handle);


//! \brief     Gets the status of the force angle operation in the estimator
//! \details   The status can only change to active when forced angle mode has been enabled by
//!            calling the following function:
//! \code
//! EST_setFlag_enableForceAngle(handle, true);
//! \endcode
//! \details   Forced angle mode will be active when the electrical frequency of the motor falls below the
//!            defined threshold in user.h:
//! \code
//! #define USER_ZEROSPEEDLIMIT (0.001)
//! \endcode
//! details    A manual check of forced angle status can be done using the following code example:
//! \code
//! _iq fe_pu = EST_getFe_pu(handle);
//! bool is_forced_angle_active;
//! if(_IQabs(fe_pu) < _IQ(USER_ZEROSPEEDLIMIT))
//!   {
//!     is_forced_angle_active = true;
//!   }
//! else
//!   {
//!     is_forced_angle_active = false;
//!   }
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether the angle has been forced (true) or not (false)
//! \retval
//!            true   The last iteration of the estimator used a forced angle to run the park and inverse park
//!                   transforms. The estimator was also run in parallel to the forced angle, but the estimator
//!                   output was not used.
//! \retval
//!            false  Forced angle mode is either disabled, or the electrical frequency did not fall below
//!                   the predetermined threshold. The estimator output was used to run the park and
//!                   inverse park transforms.
extern bool EST_getForceAngleStatus(EST_Handle handle);


//! \brief     Gets the low pass filter numerator value in the frequency estimator in per unit (pu), IQ30. 
//! \param[in] handle  The estimator (EST) handle
//! \return    The low pass filter numerator value, pu
extern _iq EST_getFreqB0_lp_pu(EST_Handle handle);


//! \brief     Gets the value used to set the pole location in the low-pass filter of the frequency estimator
//!            in per unit (pu), IQ30.
//! \param[in] handle  The estimator (EST) handle
//! \return    The value used to set the filter pole location, pu
extern _iq EST_getFreqBeta_lp_pu(EST_Handle handle);


//! \brief     Gets the slip frequency of the motor in Hertz (Hz).
//! \details   When running a permanent magnet motor, the slip frequency returned by this function will be zero.
//!            If an induction motor is used, this function will return the slip frequency. This frequency, in Hz,
//!            will be the difference between the electrical frequency and the mechanical frequency.\n
//!            \f[F_{slip}=F_{electrical}-F_{mechanical}\f]
//!
//! \param[in] handle  The estimator (EST) handle
//! \return    The slip frequency, Hz
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFslip(EST_Handle handle);
#else
extern float_t EST_getFslip(EST_Handle handle);
#endif


//! \brief     Gets the slip frequency of the motor in per unit (pu), IQ24.
//! \details   Similar to EST_getFe_pu() function, this function returns the slip frequency
//!            of the motor in per units. In order to convert the slip frequency from from per units to Hz,
//!            the user needs to multiply the returned value by the following scale factor:
//! \code
//! #define USER_IQ_FULL_SCALE_FREQ_Hz (500.0)
//!
//! _iq Full_Scale_Freq_Elec_Hz = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz);
//!
//! _iq Freq_Slip_Hz = _IQmpy(EST_getFslip_pu(handle),Full_Scale_Freq_Elec_Hz);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The slip frequency, pu
extern _iq EST_getFslip_pu(EST_Handle handle);


//! \brief     Gets the full scale current value used in the estimator in Amperes (A).
//! \details   The value returned by this function is the same as the value defined in user.h
//!            When users require to display a value in real world units, i.e. in Amperes, this
//!            value is used to convert the per unit values of currents into Amperes. The following
//!            example shows two different ways of doing this conversion, one using floating point,
//!            and the other one using IQ math.\n\n
//!            Example using floating point:
//! \code
//! float_t pu_to_amps_sf = EST_getFullScaleCurrent(handle);
//! _iq Id_rated_pu = EST_getIdRated_pu(handle);
//! float_t Id_rated_A = _IQtoF(Id_rated_pu) * pu_to_amps_sf;
//! \endcode
//! \details   Example using fixed point:
//! \code
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//!
//! _iq pu_to_amps_sf = _IQ(USER_IQ_FULL_SCALE_CURRENT_A);
//! _iq Id_rated_pu = EST_getIdRated_pu(handle);
//! _iq Id_rated_A = _IQmpy(Id_rated_pu, pu_to_amps_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The full scale current value, A
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFullScaleCurrent(EST_Handle handle);
#else
extern float_t EST_getFullScaleCurrent(EST_Handle handle);
#endif


//! \brief     Gets the full scale flux value used in the estimator in Volts per Hertz (V/Hz).
//! \param[in] handle  The estimator (EST) handle
//! \return    The full scale flux value
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFullScaleFlux(EST_Handle handle);
#else
extern float_t EST_getFullScaleFlux(EST_Handle handle);
#endif


//! \brief     Gets the full scale frequency value used in the estimator in Hertz (Hz).
//! \details   Full scale frequency can be used as a scale factor to convert values from per units to Hertz.
//!            The following example shows how to use this function to convert frequency from per units to
//!            Hz using floating point math:
//! \code
//! float_t Mechanical_Frequency_pu = _IQtoF(EST_getFm_pu(handle));
//! float_t pu_to_hz_sf = EST_getFullScaleFreq(handle);
//! float_t Mechanical_Frequency_hz = Mechanical_Frequency_pu * pu_to_hz_sf
//! \endcode
//! \details   For faster execution, this function call can be avoided by using a definition of the full
//!            scale frequency that resides in user.h. The following example shows the same functionality
//!            but using fixed point math for faster execution:
//! \code
//! #define USER_IQ_FULL_SCALE_FREQ_Hz (500.0)
//!
//! _iq Mechanical_Frequency_pu = EST_getFm_pu(handle);
//! _iq pu_to_khz_sf = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz/1000.0);
//! _iq Mechanical_Frequency_khz = _IQmpy(Mechanical_Frequency_pu, pu_to_khz_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The full scale frequency value, Hz
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFullScaleFreq(EST_Handle handle);
#else
extern float_t EST_getFullScaleFreq(EST_Handle handle);
#endif


//! \brief     Gets the full scale inductance value used in the estimator in Henries (H).
//! \details   There are different ways of getting the inductance used by the estimator. This function
//!            helps when converting an inductance from per units to H. However, the returned value is in
//!            floating point format, so utilizing this full scale value to convert per units to H is not
//!            the most efficient way. Two examples are provided below, showing a floating point per units
//!            to H conversion, and a fixed point per units to H conversion for faster execution.\n
//!            Floating point example:
//! \code
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_d_pu = _IQ30toF(EST_getLs_d_pu(handle));
//! float_t pu_to_h_sf = fullScaleInductance * pow(2.0, 30 - Ls_qFmt);
//! float_t Ls_d_H = Ls_d_pu * pu_to_h_sf;
//! \endcode
//! \details   Another example is to avoid using floating point math for faster execution. In this example
//!            the full scale inductance value is calculated using pre-compiler math based on user's parameters
//!            in user.h:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! _iq fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! _iq Ls_d_pu = _IQ30toIQ(EST_getLs_d_pu(handle));
//! _iq pu_to_h_sf = VarShift(fullScaleInductance, 30 - Ls_qFmt);
//! _iq Ls_d_H = _IQmpy(Ls_d_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The full scale resistance value, Henry
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFullScaleInductance(EST_Handle handle);
#else
extern float_t EST_getFullScaleInductance(EST_Handle handle);
#endif


//! \brief     Gets the full scale resistance value used in the estimator in Ohms (\f$\Omega\f$).
//! \details   There are different ways of getting the resistance used by the estimator. This function
//!            helps when converting resistance from per units to Ohms. However, the returned value is in
//!            floating point format, so utilizing this full scale value to convert per units to Ohms is not
//!            the most efficient way. Two examples are provided below, showing a floating point per units
//!            to Ohms conversion, and a fixed point per units to Ohms conversion for faster execution.\n
//!            Floating point example:
//! \code
//! uint_least8_t Rs_qFmt = EST_getRs_qFmt(handle);
//! float_t fullScaleResistance = EST_getFullScaleResistance(handle);
//! float_t Rs_pu = _IQ30toF(EST_getRs_pu(handle));
//! float_t pu_to_ohms_sf = fullScaleResistance * pow(2.0, 30 - Rs_qFmt);
//! float_t Rs_Ohms = Rs_pu * pu_to_ohms_sf;
//! \endcode
//! \details   Another example is to avoid using floating point math for faster execution. In this example
//!            the full scale resistance value is calculated using pre-compiler math based on user's parameters
//!            in user.h:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//!
//! uint_least8_t Rs_qFmt = EST_getRs_qFmt(handle);
//! _iq fullScaleResistance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/USER_IQ_FULL_SCALE_CURRENT_A);
//! _iq Rs_pu = _IQ30toIQ(EST_getRs_pu(handle));
//! _iq pu_to_ohms_sf = VarShift(fullScaleResistance, 30 - Rs_qFmt);
//! _iq Rs_Ohms = _IQmpy(Rs_pu, pu_to_ohms_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The full scale resistance value, Ohm
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFullScaleResistance(EST_Handle handle);
#else
extern float_t EST_getFullScaleResistance(EST_Handle handle);
#endif


//! \brief     Gets the full scale voltage value used in the estimator in Volts (V).
//! \details   The value returned by this function is the same as the value defined in user.h
//!            When users require to display a value in real world units, i.e. in Volts, this
//!            value is used to convert the per unit values of voltage into Volts. The following
//!            example shows two different ways of doing this conversion, one using floating point,
//!            and the other one using IQ math.\n\n
//!            Example using floating point:
//! \code
//! float_t pu_to_v_sf = EST_getFullScaleVoltage(handle);
//! _iq DcBus_pu = EST_getDcBus_pu(handle);
//! float_t DcBus_V = _IQtoF(DcBus_pu) * pu_to_v_sf;
//! \endcode
//! \details   Example using fixed point:
//! \code
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//!
//! _iq pu_to_kv_sf = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0);
//! _iq DcBus_pu = EST_getDcBus_pu(handle);
//! _iq DcBus_kV = _IQmpy(DcBus_pu, pu_to_kv_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The full scale voltage value, V
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getFullScaleVoltage(EST_Handle handle);
#else
extern float_t EST_getFullScaleVoltage(EST_Handle handle);
#endif


//! \brief     Gets the Iab current vector in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pIab    The pointer to memory for the Iab vector, pu
extern void EST_getIab_pu(EST_Handle handle,MATH_vec2 *pIab);


//! \brief     Gets the Id rated current value from the estimator in Amperes (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated current value, A
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getIdRated(EST_Handle handle);
#else
extern float_t EST_getIdRated(EST_Handle handle);
#endif


//! \brief     Gets the Id rated current value from the estimator in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated current value, pu
extern _iq EST_getIdRated_pu(EST_Handle handle);


//! \brief     Gets the Id current value used for inductance estimation of induction motors
//!            in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated value, pu
extern _iq EST_getIdRated_indEst_pu(EST_Handle handle);


//! \brief     Gets the Id current value used for flux estimation of induction motors in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated value, pu
extern _iq EST_getIdRated_ratedFlux_pu(EST_Handle handle);


//! \brief     Gets the rotor inductance value in Henries (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor inductance value, Henry
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getLr_H(EST_Handle handle);
#else
extern float_t EST_getLr_H(EST_Handle handle);
#endif


//! \brief     Gets the rotor inductance value in per unit (pu), IQ30.
//! \details   The per units value of the rotor inductance can be used as an alternative way
//!            of calculating the rotor inductance of an induction motor using fixed point math.
//!            An example showing how this is done is shown here:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Lr_qFmt = EST_getLr_qFmt(handle);
//! _iq fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! _iq Lr_pu = _IQ30toIQ(EST_getLr_pu(handle));
//! _iq pu_to_h_sf = VarShift(fullScaleInductance, 30 - Lr_qFmt);
//! _iq Lr_H = _IQmpy(Lr_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor inductance value, pu
extern _iq EST_getLr_pu(EST_Handle handle);


//! \brief     Gets the rotor inductance Q format in 8 bit unsigned integer (uint_least8_t).
//! \details   When the motor is identified by the estimator, the Q format is used to have a wider
//!            range of the identified parameter. This Q format is the difference between the actual Q
//!            format used for the identification and IQ30 which is used internaly during identification
//!            of the motor parameters. To understand how this Q format can be used in user's code, please
//!            refer to the following example, which converts a per units value read from the estimator
//!            to Henries:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Lr_qFmt = EST_getLr_qFmt(handle);
//! _iq fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! _iq Lr_pu = _IQ30toIQ(EST_getLr_pu(handle));
//! _iq pu_to_h_sf = VarShift(fullScaleInductance, 30 - Lr_qFmt);
//! _iq Lr_H = _IQmpy(Lr_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor inductance value Q format
extern uint_least8_t EST_getLr_qFmt(EST_Handle handle);


//! \brief     Gets the direct stator inductance value in Henries (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator inductance value
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getLs_d_H(EST_Handle handle);
#else
extern float_t EST_getLs_d_H(EST_Handle handle);
#endif


//! \brief     Gets the direct stator inductance value in per unit (pu), IQ30.
//! \details   The per units value of the direct stator inductance can be used as an alternative way
//!            of calculating the direct stator inductance of a permanent magnet motor using fixed point math.
//!            An example showing how this is done is shown here:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! _iq fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! _iq Ls_d_pu = _IQ30toIQ(EST_getLs_d_pu(handle));
//! _iq pu_to_h_sf = VarShift(fullScaleInductance, 30 - Ls_qFmt);
//! _iq Ls_d_H = _IQmpy(Ls_d_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator inductance value, pu
extern _iq EST_getLs_d_pu(EST_Handle handle);


//! \brief     Gets the direct/quadrature stator inductance vector values from the estimator in
//!            per unit (pu), IQ30.
//! \details   Both direct and quadrature stator inductances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pLs_dq_pu   The pointer for the direct/quadrature stator inductance vector values, pu
extern void EST_getLs_dq_pu(EST_Handle handle,MATH_vec2 *pLs_dq_pu);


//! \brief     Gets the stator inductance value in the quadrature coordinate direction in Henries (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getLs_q_H(EST_Handle handle);
#else
extern float_t EST_getLs_q_H(EST_Handle handle);
#endif


//! \brief     Gets the stator inductance value in the quadrature coordinate direction in per unit (pu), IQ30.
//! \details   The per units value of the quadrature stator inductance can be used as an alternative way
//!            of calculating the quadrature stator inductance of a permanent magnet motor using fixed point math.
//!            An example showing how this is done is shown here:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! _iq fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! _iq Ls_q_pu = _IQ30toIQ(EST_getLs_q_pu(handle));
//! _iq pu_to_h_sf = VarShift(fullScaleInductance, 30 - Ls_qFmt);
//! _iq Ls_q_H = _IQmpy(Ls_q_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value, pu
extern _iq EST_getLs_q_pu(EST_Handle handle);


//! \brief     Gets the stator inductance Q format in 8 bit unsigned integer (uint_least8_t).
//! \details   When the motor is identified by the estimator, the Q format is used to have a wider
//!            range of the identified parameter. This Q format is the difference between the actual Q
//!            format used for the identification and IQ30 which is used internaly during identification
//!            of the motor parameters. To understand how this Q format can be used in user's code, please
//!            refer to the following example, which converts a per units value read from the estimator
//!            to Henries:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! _iq fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! _iq Ls_q_pu = _IQ30toIQ(EST_getLs_q_pu(handle));
//! _iq pu_to_h_sf = VarShift(fullScaleInductance, 30 - Ls_qFmt);
//! _iq Ls_q_H = _IQmpy(Ls_q_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance Q format
extern uint_least8_t EST_getLs_qFmt(EST_Handle handle);


//! \brief     Gets the maximum stator inductance value from the stator inductance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum stator inductance value, pu
extern _iq EST_getLs_max_pu(EST_Handle handle);


//! \brief     Gets the minimum stator inductance value from the stator inductance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The minimum stator inductance value, pu
extern _iq EST_getLs_min_pu(EST_Handle handle);


//! \brief     Gets the maximum stator inductance value during coarse estimation in the stator inductance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum stator inductance value, pu
extern _iq EST_getLs_coarse_max_pu(EST_Handle handle);


//! \brief     Gets the maximum acceleration value used in the estimator in per unit (pu), IQ24.
//! \details   The maximum acceleration is a setting of the trajectory module, which sets the speed reference.
//!            The acceleration returned by this function call is used after the motor has been identified.
//!            This value represents how the speed reference is increased or decreased from an initial value
//!            to a target value. The following example shows how convert the returned value of this function
//!            to kilo Hertz per second (kHz/s) and kilo RPM per second (kRPM/s):
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz        (500.0)
//! #define USER_MOTOR_NUM_POLE_PAIRS         (4)
//!
//! _iq pu_to_khzps_sf = _IQ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz / 1000.0);
//! _iq khzps_to_krpmps_sf = _IQ(60.0 / (float_t)USER_MOTOR_NUM_POLE_PAIRS);
//! 
//! _iq Accel_pu = EST_getMaxAccel_pu(handle);
//! _iq Accel_kilo_hz_per_sec = _IQmpy(Accel_pu, pu_to_khzps_sf);
//! _iq Accel_kilo_rpm_per_sec = _IQmpy(Accel_kilo_hz_per_sec, khzps_to_krpmps_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz        (500.0)
//! #define USER_MAX_ACCEL_Hzps               (20.0)
//!
//! _iq hzps_to_pu_sf = _IQ(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz));
//! 
//! _iq Accel_hertz_per_sec = _IQ(USER_MAX_ACCEL_Hzps);
//! _iq Accel_pu = _IQmpy(Accel_hertz_per_sec, hzps_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum acceleration value, pu
extern _iq EST_getMaxAccel_pu(EST_Handle handle);


//! \brief     Gets the maximum estimation acceleration value used in the estimator in per unit (pu), IQ24.
//! \details   The maximum acceleration is a setting of the trajectory module, which sets the speed reference.
//!            The acceleration returned by this function call is used during the motor identification process.
//!            This value represents how the speed reference is increased or decreased from an initial value
//!            to a target value. The following example shows how convert the returned value of this function
//!            to kilo Hertz per Second (kHz/s) and kilo RPM per second (kRPM/s):
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz        (500.0)
//! #define USER_MOTOR_NUM_POLE_PAIRS         (4)
//!
//! _iq pu_to_khzps_sf = _IQ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz / 1000.0);
//! _iq khzps_to_krpmps_sf = _IQ(60.0 / (float_t)USER_MOTOR_NUM_POLE_PAIRS);
//! 
//! _iq est_Accel_pu = EST_getMaxAccel_est_pu(handle);
//! _iq est_Accel_kilo_hz_per_sec = _IQmpy(est_Accel_pu, pu_to_khzps_sf);
//! _iq est_Accel_kilo_rpm_per_sec = _IQmpy(est_Accel_kilo_hz_per_sec, khzps_to_krpmps_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz        (500.0)
//! #define USER_MAX_ACCEL_EST_Hzps           (2.0)
//!
//! _iq hzps_to_pu_sf = _IQ(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz));
//! 
//! _iq est_Accel_hertz_per_sec = _IQ(USER_MAX_ACCEL_EST_Hzps);
//! _iq est_Accel_pu = _IQmpy(est_Accel_hertz_per_sec, hzps_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum estimation acceleration value, pu
extern _iq EST_getMaxAccel_est_pu(EST_Handle handle);


//! \brief     Gets the maximum current slope value used in the estimator in per unit (pu), IQ24.
//! \details   Gets the slope of Id reference. The following example shows how to convert the returned
//!            value into kilo Amperes per second (kA/s):
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_CURRENT_A      (10.0)
//!
//! _iq pu_to_kA_per_sec_sf = _IQ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_CURRENT_A / 1000.0);
//!
//! _iq currentSlope_pu = EST_getMaxCurrentSlope_pu(handle);
//! _iq currentSlope_kAps = _IQmpy(currentSlope_pu, pu_to_kA_per_sec_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_CURRENT_A      (10.0)
//! #define USER_MOTOR_RES_EST_CURRENT        (1.0)
//!
//! _iq A_per_sec_to_pu_sf = _IQ(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_CURRENT_A));
//!
//! _iq currentSlope_Aps = _IQ(USER_MOTOR_RES_EST_CURRENT);
//! _iq currentSlope_pu = _IQmpy(currentSlope_Aps, A_per_sec_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum current slope value, pu
extern _iq EST_getMaxCurrentSlope_pu(EST_Handle handle);


//! \brief     Gets the maximum EPL (Efficient Partial Load) current slope value used in the estimator
//!            in per unit (pu), IQ24.
//! \details   Gets the slope of Id reference change when efficient partial load is enabled. This mode only
//!            applies to induction motors. The following example shows how to convert the returned value into 
//!            kilo Amperes per second (kA/s):
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_CURRENT_A      (10.0)
//!
//! _iq pu_to_kA_per_sec_sf = _IQ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_CURRENT_A / 1000.0);
//!
//! _iq currentSlope_epl_pu = EST_getMaxCurrentSlope_epl_pu(handle);
//! _iq currentSlope_epl_kAps = _IQmpy(currentSlope_epl_pu, pu_to_kA_per_sec_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USER_IQ_FULL_SCALE_CURRENT_A      (10.0)
//! #define USER_MOTOR_RES_EST_CURRENT        (1.0)
//!
//! _iq A_per_sec_to_pu_sf = _IQ(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_CURRENT_A));
//!
//! _iq currentSlope_epl_Aps = _IQ(0.3 * USER_MOTOR_RES_EST_CURRENT);
//! _iq currentSlope_epl_pu = _IQmpy(currentSlope_epl_Aps, A_per_sec_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum EPL current slope value, pu
extern _iq EST_getMaxCurrentSlope_epl_pu(EST_Handle handle);


//! \brief     Gets the inverse of the DC bus voltage in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The inverse of the DC bus voltage, pu
extern _iq EST_getOneOverDcBus_pu(EST_Handle handle);


//! \brief     Gets the rotor resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor resistance value, Ohm
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getRr_Ohm(EST_Handle handle);
#else
extern float_t EST_getRr_Ohm(EST_Handle handle);
#endif


//! \brief     Gets the rotor resistance value in per unit (pu), IQ30.
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor resistance value, pu
extern _iq EST_getRr_pu(EST_Handle handle);


//! \brief     Gets the rotor resistance Q format in 8 bit unsigned integer (uint_least8_t).
//! \param[in] handle  The estimator (EST) handle
//! \return    The rotor resistance Q format
extern uint_least8_t EST_getRr_qFmt(EST_Handle handle);


//! \brief     Gets the stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator resistance value, Ohm
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getRs_Ohm(EST_Handle handle);
#else
extern float_t EST_getRs_Ohm(EST_Handle handle);
#endif


//! \brief     Gets the stator resistance value in per unit (pu), IQ30.
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator resistance value, pu
extern _iq EST_getRs_pu(EST_Handle handle);


//! \brief     Gets the stator resistance Q format in 8 bit unsigned integer (uint_least8_t).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator resistance Q format
extern uint_least8_t EST_getRs_qFmt(EST_Handle handle);


//! \brief     Gets the online stator resistance filter parameters in per unit (pu), IQ24.
//! \param[in] handle        The estimator (EST) handle
//! \param[in] filterType    The filter type
//! \param[in] pFilter_0_b0  The pointer for the filter 0 numerator coefficient value for z^0
//! \param[in] pFilter_0_a1  The pointer for the filter 0 denominator coefficient value for z^(-1)
//! \param[in] pFilter_0_y1  The pointer for the filter 0 output value at time sample n=-1
//! \param[in] pFilter_1_b0  The pointer for the filter 1 numerator coefficient value for z^0
//! \param[in] pFilter_1_a1  The pointer for the filter 1 denominator coefficient value for z^(-1)
//! \param[in] pFilter_1_y1  The pointer for the filter 1 output value at time sample n=-1
extern void EST_getRsOnLineFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                        _iq *pFilter_0_b0,_iq *pFilter_0_a1,_iq *pFilter_0_y1,
                                        _iq *pFilter_1_b0,_iq *pFilter_1_a1,_iq *pFilter_1_y1);


//! \brief     Gets the online stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The online stator resistance value, Ohm
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getRsOnLine_Ohm(EST_Handle handle);
#else
extern float_t EST_getRsOnLine_Ohm(EST_Handle handle);
#endif


//! \brief     Gets the online stator resistance value in per unit (pu), IQ30.
//! \param[in] handle  The estimator (EST) handle
//! \return    The online stator resistance value, pu
extern _iq EST_getRsOnLine_pu(EST_Handle handle);


//! \brief     Gets the online stator resistance Q format in 8 bit unsigned integer (uint_least8_t).
//! \param[in] handle  The estimator (EST) handle
//! \return    The online stator resistance Q format
extern uint_least8_t EST_getRsOnLine_qFmt(EST_Handle handle);


//! \brief     Gets the Id magnitude value used for online stator resistance estimation in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id magnitude value, pu
extern _iq EST_getRsOnLineId_mag_pu(EST_Handle handle);


//! \brief     Gets the Id value used for online stator resistance estimation in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id value, pu
extern _iq EST_getRsOnLineId_pu(EST_Handle handle);


//! \brief     Sets the Id value in the online stator resistance estimator in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Id_pu   The Id value, pu
extern void EST_setRsOnLineId_pu(EST_Handle handle,const _iq Id_pu);


//! \brief     Gets the sign of the direction value in 8 bit signed integer (int_least8_t).
//! \param[in] handle  The estimator (EST) handle
//! \return    The sign of the direction value (-1 for negative, 1 for positive)
extern int_least8_t EST_getSignOfDirection(EST_Handle handle);


//! \brief     Gets the speed value in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The speed value, krpm
extern _iq EST_getSpeed_krpm(EST_Handle handle);


//! \brief     Gets the state of the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimator state
extern EST_State_e EST_getState(EST_Handle handle);


//! \brief     Gets the torque value in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The torque value, lb*in
extern _iq EST_getTorque_lbin(EST_Handle handle);


//! \brief     Gets the torque value in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The torque value, N*m
extern _iq EST_getTorque_Nm(EST_Handle handle);


//! \brief     Determines if there is an estimator error
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if there is an estimator error (true) or not (false)
extern bool EST_isError(EST_Handle handle);


//! \brief     Determines if the estimator is idle
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is idle (true) or not (false)
extern bool EST_isIdle(EST_Handle handle);


//! \brief     Determines if the estimator is waiting for the rotor to be locked
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is waiting for the rotor to be locked (true) or not (false)
extern bool EST_isLockRotor(EST_Handle handle);


//! \brief     Determines if the motor has been identified
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the motor is identified (true) or not (false)
extern bool EST_isMotorIdentified(EST_Handle handle);


//! \brief     Determines if the estimator is ready for online control
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is ready for online control (true) or not (false)
extern bool EST_isOnLine(EST_Handle handle);


//! \brief     Resets the control counter
//! \param[in] handle  The estimator (EST) handle
extern void EST_resetCounter_ctrl(EST_Handle handle);


//! \brief     Resets the state counter
//! \param[in] handle  The estimator (EST) handle
extern void EST_resetCounter_state(EST_Handle handle);


//! \brief     Runs the estimator
//! \param[in] handle         The estimator (EST) handle
//! \param[in] pIab_pu        The pointer to the phase currents in the alpha/beta coordinate system, pu IQ24
//! \param[in] pVab_pu        The pointer to the phase voltages in the alpha/beta coordinate system, pu IQ24
//! \param[in] dcBus_pu       The DC bus voltage, pu IQ24
//! \param[in] speed_ref_pu   The speed reference value to the controller, pu IQ24
extern void EST_run(EST_Handle handle,
             const MATH_vec2 *pIab_pu,
             const MATH_vec2 *pVab_pu,
             const _iq dcBus_pu,
             const _iq speed_ref_pu);


//! \brief     Sets the angle value in the estimator in per unit (pu), IQ24.
//! \details   This function overwrites the estimated angle with a user's provided angle. 
//!            The set value should be between 0x00000000 or _IQ(0.0) to 0x00FFFFFF or _IQ(1.0). 
//!            The following example shows how to overwrite the estimated angle:
//! \code
//! _iq Overwrite_Flux_Angle_pu = _IQ(0.5);
//! EST_setAngle_pu(handle, Overwrite_Flux_Angle_pu);
//! \endcode
//! \details   This function is not recommended for general use, since this will automatically generate
//!            an axis misalignment between the rotor flux axis and the control signals driving the motor.
//!            The use of this function is recommended for advanced users interested in doing open loop 
//!            startup algorithms that need to bypass the estimator.
//! \param[in] handle    The estimator (EST) handle
//! \param[in] angle_pu  The angle value, pu
extern void EST_setAngle_pu(EST_Handle handle,const _iq angle_pu);


//! \brief     Sets the DC bus voltage in the estimator in per unit (pu), IQ24.
//! \param[in] handle    The estimator (EST) handle
//! \param[in] dcBus_pu  The DC bus voltage, pu
extern void EST_setDcBus_pu(EST_Handle handle,const _iq dcBus_pu);


//! \brief     Sets the enable flux control flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
extern void EST_setFlag_enableFluxControl(EST_Handle handle,const bool state);


//! \brief     Sets the enable force angle flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable forced angle. The estimated angle will be bypassed if the flux
//!                    frequency falls below a threashold defined by:
//!                    \code #define USER_ZEROSPEEDLIMIT (0.001) \endcode
//!                    in user.h. A typical value of this frequency is 0.001 of the full scale frequency
//!                    defined in:
//!                    \code #define USER_IQ_FULL_SCALE_FREQ_Hz (500.0) \endcode
//!                    Forced angle algorithm, when active, that is, when the rotor flux electrical
//!                    frequency falls below the threashold, will be forcing a rotating angle at a
//!                    frequency set by the following define:
//!                    \code #define USER_FORCE_ANGLE_FREQ_Hz  (1.0) \endcode
//!            <b>false</b> Disable forced angle. The estimator will never be bypassed by any forced
//!                    angle algorithm.
extern void EST_setFlag_enableForceAngle(EST_Handle handle,const bool state);


//! \brief     Sets the enable Rs online flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable the Rs online recalibration algorithm. The estimator will run a set of
//!                    functions related to rs online which recalculates the stator resistance while the
//!                    motor is rotating. This algorithm is useful when motor heats up, and hence stator
//!                    resistance increases.
//!            <b>false</b> Disable the Rs online recalibration algorithm. No updates to Rs will be made
//!                    even if the motor heats up. Low speed performace, and startup performance with
//!                    full torque might be affected if stator resistance changes due to motor heating up.
//!                    The stator resistance will be fixed, and equal to the value returned by EST_getRs_Ohm().
extern void EST_setFlag_enableRsOnLine(EST_Handle handle,const bool state);


//! \brief     Sets the enable stator resistance (Rs) re-calculation flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable Rs recalibration. The estimator will inject a DC current to the D-axis
//!                    to recalibrate the stator resistance before the motor rotates. It is required that
//!                    the motor is at standstill to perform Rs recalibration. If online recalibration
//!                    of the stator resistance is needed, refer to EST_getFlag_enableRsOnLine() and
//!                    EST_setFlag_enableRsOnLine() functions.
//!            <b>false</b> Disable Rs recalibration. The estimator will start the motor with the resistance
//!                    value that was used before the motor was stopped, or what is returned by function:
//!                    EST_getRs_Ohm().
extern void EST_setFlag_enableRsRecalc(EST_Handle handle,const bool state);


//! \brief     Sets the estimation complete flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, true (1) or false (0)
extern void EST_setFlag_estComplete(EST_Handle handle,const bool state);


//! \brief     Sets the update stator resistance (Rs) flag in the estimator
//! \details   When the online resistance estimator is enabled, the update flag allows the online resistance
//!            to be copied to the resistance used by the estimator model. If the update flag is not set to true,
//!            the online resistance estimation will not be used by the estimator model, and if the resistance
//!            changes too much due to temperature increase, the model may not work as expected.
//! \code
//! EST_setFlag_updateRs(handle, true);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state
//!            <b>true</b> The stator resistance estimated by the Rs OnLine module will be copied to the'
//!                    the stator resistance used by the module, so of the motor's temperature changes,
//!                    the estimated angle will be calculated based on the most up to date stator
//!                    resistance
//!            <b>false</b> The stator resistance estimated by the Rs OnLine module may or may not be updated
//!                    depending on the enable flag, but will not be used in the motor's model used to generate
//!                    the estimated speed and angle.
extern void EST_setFlag_updateRs(EST_Handle handle,const bool state);


//! \brief     Sets the force angle delta value in the estimator in per unit (pu), IQ24.
//! \details   This function sets a forced angle delta, which represents the increments
//!            to be added to or subtracted from the forced angle. The higher this value is, the higher
//!            frequency will be generated when the angle is forced (estimated angle is bypassed when 
//!            in forced angle mode). By default the forced angle frequency is set in user.h. 
//!            The following example shows how to set a forced angle frequency from Hertz (Hz) to per unit:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_EST_TICK  (1)
//! #define USER_PWM_FREQ_kHz         (15.0)
//! #define USER_ISR_FREQ_Hz          (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz         (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_EST_FREQ_Hz          (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)
//! #define USER_FORCE_ANGLE_FREQ_Hz  (1.0)
//!
//! _iq delta_hz_to_pu_sf = _IQ(1.0/(float_t)USER_EST_FREQ_Hz);
//! _iq Force_Angle_Freq_Hz = _IQ(USER_FORCE_ANGLE_FREQ_Hz);
//! _iq Force_Angle_Delta_pu = _IQmpy(Force_Angle_Freq_Hz, delta_hz_to_pu_sf);
//!
//! EST_setForceAngleDelta_pu(handle, Force_Angle_Delta_pu);
//! \endcode
//! \param[in] handle         The estimator (EST) handle
//! \param[in] angleDelta_pu  The force angle delta value, pu
extern void EST_setForceAngleDelta_pu(EST_Handle handle,const _iq angleDelta_pu);


//! \brief     Sets the low pass filter numerator value in the frequency estimator in per unit (pu), IQ30.
//! \param[in] handle    The estimator (EST) handle
//! \param[in] b0_lp_pu  The low pass filter numerator value, pu
extern void EST_setFreqB0_lp_pu(EST_Handle handle,const _iq b0_lp_pu);


//! \brief     Sets the value used to set the low pass pole location in the frequency estimator
//!            in per unit (pu), IQ30.
//! \param[in] handle      The estimator (EST) handle
//! \param[in] beta_lp_pu  The value used to set the filter pole location, pu
extern void EST_setFreqBeta_lp_pu(EST_Handle handle,const _iq beta_lp_pu);


//! \brief     Sets the full scale current in the estimator in Amperes (A).
//! \param[in] handle            The estimator (EST) handle
//! \param[in] fullScaleCurrent  The full scale current, A
extern void EST_setFullScaleCurrent(EST_Handle handle,const float_t fullScaleCurrent);


//! \brief     Gets the full scale flux value used in the estimator in Volts*seconds (V.s).
//! \param[in] handle         The estimator (EST) handle
//! \param[in] fullScaleFlux  The full scale flux value, V*sec
extern void EST_setFullScaleFlux(EST_Handle handle,const float_t fullScaleFlux);


//! \brief     Sets the full scale frequency in the estimator in Hertz (Hz).
//! \param[in] handle         The estimator (EST) handle
//! \param[in] fullScaleFreq  The full scale frequency, Hz
extern void EST_setFullScaleFreq(EST_Handle handle,const float_t fullScaleFreq);


//! \brief     Sets the full scale inductance in the estimator in Henries (H).
//! \param[in] handle               The estimator (EST) handle
//! \param[in] fullScaleInductance  The full scale inductance, Henry
extern void EST_setFullScaleInductance(EST_Handle handle,const float_t fullScaleInductance);


//! \brief     Sets the full scale resistance in the estimator in Ohms (\f$\Omega\f$).
//! \param[in] handle               The estimator (EST) handle
//! \param[in] fullScaleResistance  The full scale resistance, Ohm
extern void EST_setFullScaleResistance(EST_Handle handle,const float_t fullScaleResistance);


//! \brief     Sets the full scale resistance in the estimator in Volts (V).
//! \param[in] handle            The estimator (EST) handle
//! \param[in] fullScaleVoltage  The full scale voltage, V
extern void EST_setFullScaleVoltage(EST_Handle handle,const float_t fullScaleVoltage);


//! \brief     Sets the estimator to idle
//! \param[in] handle  The estimator (EST) handle
extern void EST_setIdle(EST_Handle handle);


//! \brief     Sets the estimator and all of the subordinate estimators to idle
//! \param[in] handle    The estimator (EST) handle
extern void EST_setIdle_all(EST_Handle handle);


//! \brief     Sets the direct current (Id) reference value in the estimator in per unit (pu), IQ24.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Id_ref_pu  The Id reference value, pu
extern void EST_setId_ref_pu(EST_Handle handle,const _iq Id_ref_pu);


//! \brief     Sets the Id rated current value in the estimator in per unit (pu), IQ24.
//! \param[in] handle      The estimator (EST) handle
//! \param[in] IdRated_pu  The Id rated current value, pu
extern void EST_setIdRated_pu(EST_Handle handle,const _iq IdRated_pu);


//! \brief     Sets the quadrature current (Iq) reference value in the estimator in per unit (pu), IQ24.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Iq_ref_pu  The Iq reference value, pu
extern void EST_setIq_ref_pu(EST_Handle handle,const _iq Iq_ref_pu);


//! \brief     Sets the direct stator inductance value in the estimator in per unit (pu), IQ30.
//! \details   The internal direct inductance (Ls_d) used by the estimator can be changed in real time 
//!            by calling this function. An example showing how this is done is shown here:
//! \code
//! #define USER_MOTOR_Ls_d  (0.012)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = ceil(log(USER_MOTOR_Ls_d/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,lShift);
//! _iq Ls_d_pu = _IQ30(USER_MOTOR_Ls_d / L_max);
//!
//! EST_setLs_d_pu(handle, Ls_d_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Ls_d_pu  The direct stator inductance value, pu
extern void EST_setLs_d_pu(EST_Handle handle,const _iq Ls_d_pu);


//! \brief     Gets the delta stator inductance value in the stator inductance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator inductance value, pu
extern _iq EST_getLs_delta_pu(EST_Handle handle);


//! \brief     Sets the delta stator inductance value during fine estimation
//! \param[in] handle       The estimator (EST) handle
//! \param[in] Ls_delta_pu  The delta stator inductance value, pu
extern void EST_setLs_delta_pu(EST_Handle handle,const _iq Ls_delta_pu);


//! \brief     Gets the delta stator resistance value from the stator resistance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator resistance value, pu
extern _iq EST_getRs_delta_pu(EST_Handle handle);


//! \brief     Sets the delta stator resistance value
//! \param[in] handle       The estimator (EST) handle
//! \param[in] Rs_delta_pu  The delta stator resistance value, pu
extern void EST_setRs_delta_pu(EST_Handle handle,const _iq Rs_delta_pu);


//! \brief     Sets the direct/quadrature stator inductance vector values in the estimator in per unit (pu), IQ30.
//! \details   The internal direct and quadrature inductances (Ls_d and Ls_q) used by the estimator can be changed
//!            in real time by calling this function. An example showing how this is done is shown here:
//! \code
//! #define USER_MOTOR_Ls_d  (0.012)
//! #define USER_MOTOR_Ls_q  (0.027)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = ceil(log(USER_MOTOR_Ls_d/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,lShift);
//! MATH_vec2 Ls_dq_pu;
//!
//! Ls_dq_pu.value[0] = _IQ30(USER_MOTOR_Ls_d / L_max);
//! Ls_dq_pu.value[1] = _IQ30(USER_MOTOR_Ls_q / L_max);
//!
//! EST_setLs_dq_pu(handle, &Ls_dq_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle     The estimator (EST) handle
//! \param[in] pLs_dq_pu  The pointer to the direct/quadrature stator inductance vector values, pu
extern void EST_setLs_dq_pu(EST_Handle handle,const MATH_vec2 *pLs_dq_pu);


//! \brief     Sets the quadrature stator inductance value in the estimator in per unit (pu), IQ30.
//! \details   The internal quadrature inductance (Ls_q) used by the estimator can be changed in real time 
//!            by calling this function. An example showing how this is done is shown here:
//! \code
//! #define USER_MOTOR_Ls_q  (0.027)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = ceil(log(USER_MOTOR_Ls_q/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,lShift);
//! _iq Ls_d_pu = _IQ30(USER_MOTOR_Ls_q / L_max);
//!
//! EST_setLs_q_pu(handle, Ls_q_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Ls_q_pu  The quadrature stator inductance value, pu
extern void EST_setLs_q_pu(EST_Handle handle,const _iq Ls_q_pu);


//! \brief     Sets the stator inductance Q format in the estimator in 8 bit unsigned integer (uint_least8_t).
//! \details   Updating the internal inductance also requires to update the Q format variable, which is used
//!            to extend the covered range. This qFmt (Q Format) variable creates a floating point using fixed
//!            point math. It is important to notice that the inductance Q Format set by calling EST_setLs_qFmt()
//!            will be used by both per unit inductance calculations Ls_d and Ls_q. An example showing how this
//!            Q Format is set is shown below:
//! \code
//! #define USER_MOTOR_Ls_d  (0.012)
//! #define USER_MOTOR_Ls_q  (0.027)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = ceil(log(USER_MOTOR_Ls_d/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,lShift);
//! MATH_vec2 Ls_dq_pu;
//!
//! Ls_dq_pu.value[0] = _IQ30(USER_MOTOR_Ls_d / L_max);
//! Ls_dq_pu.value[1] = _IQ30(USER_MOTOR_Ls_q / L_max);
//!
//! EST_setLs_dq_pu(handle, &Ls_dq_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Ls_qFmt  The stator inductance Q format
extern void EST_setLs_qFmt(EST_Handle handle,const uint_least8_t Ls_qFmt);


//! \brief     Sets the maximum acceleration value in the estimator in per unit (pu), IQ24.
//! \param[in] handle       The estimator (EST) handle
//! \param[in] maxAccel_pu  The maximum acceleration value, pu
extern void EST_setMaxAccel_pu(EST_Handle handle,const _iq maxAccel_pu);


//! \brief     Sets the maximum estimation acceleration value in the estimator in per unit (pu), IQ24.
//! \param[in] handle       The estimator (EST) handle
//! \param[in] maxAccel_pu  The maximum estimation acceleration value, pu
extern void EST_setMaxAccel_est_pu(EST_Handle handle,const _iq maxAccel_pu);


//! \brief     Sets the maximum current slope value in the estimator in per unit (pu), IQ24.
//! \param[in] handle              The estimator (EST) handle
//! \param[in] maxCurrentSlope_pu  The maximum current slope value, pu
extern void EST_setMaxCurrentSlope_pu(EST_Handle handle,const _iq maxCurrentSlope_pu);


//! \brief     Sets the maximum EPL (Efficient Partial Load) current slope value used in the estimator
//!            in per unit (pu), IQ24.
//! \param[in] handle              The estimator (EST) handle
//! \param[in] maxCurrentSlope_pu  The maximum current slope value, pu
extern void EST_setMaxCurrentSlope_epl_pu(EST_Handle handle,const _iq maxCurrentSlope_pu);


//! \brief     Sets the rotor resistance value in the estimator in per unit (pu), IQ30.
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rr_pu    The rotor resistance value, pu
extern void EST_setRr_pu(EST_Handle handle,const _iq Rr_pu);


//! \brief     Sets the rotor resistance Q format in the estimator in 8 bit unsigned integer (uint_least8_t).
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rr_qFmt  The rotor resistance Q format
extern void EST_setRr_qFmt(EST_Handle handle,uint_least8_t Rr_qFmt);


//! \brief     Sets the delta angle value in the online stator resistance estimator
//! \param[in] handle         The estimator (EST) handle
//! \param[in] angleDelta_pu  The delta angle value, pu
extern void EST_setRsOnLineAngleDelta_pu(EST_Handle handle,const _iq angleDelta_pu);


//! \brief     Sets the stator resistance value in the online stator resistance estimator in per unit (pu), IQ30.
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Rs_pu   The stator resistance value, pu
extern void EST_setRsOnLine_pu(EST_Handle handle,const _iq Rs_pu);


//! \brief     Sets the stator resistance Q format in the online stator resistance estimator
//!            in 8 bit unsigned integer (uint_least8_t).
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rs_qFmt  The stator resistance Q format
extern void EST_setRsOnLine_qFmt(EST_Handle handle,const uint_least8_t Rs_qFmt);


//! \brief     Sets the online stator resistance filter parameters in per unit (pu), IQ24.
//! \param[in] handle       The estimator (EST) handle
//! \param[in] filterType   The filter type
//! \param[in] filter_0_b0  The filter 0 numerator coefficient value for z^0
//! \param[in] filter_0_a1  The filter 0 denominator coefficient value for z^(-1)
//! \param[in] filter_0_y1  The filter 0 output value at time sample n=-1
//! \param[in] filter_1_b0  The filter 1 numerator coefficient value for z^0
//! \param[in] filter_1_a1  The filter 1 denominator coefficient value for z^(-1)
//! \param[in] filter_1_y1  The filter 1 output value at time sample n=-1
extern void EST_setRsOnLineFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                        const _iq filter_0_b0,const _iq filter_0_a1,const _iq filter_0_y1,
                                        const _iq filter_1_b0,const _iq filter_1_a1,const _iq filter_1_y1);


//! \brief     Sets the Id magnitude value used for online stator resistance estimation in per unit (pu), IQ24.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Id_mag_pu  The Id magnitude value, pu
extern void EST_setRsOnLineId_mag_pu(EST_Handle handle,const _iq Id_mag_pu);


//! \brief     Sets the stator resistance value used in the estimator in per unit (pu), IQ30.
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rs_pu    The stator resistance value, pu
extern void EST_setRs_pu(EST_Handle handle,const _iq Rs_pu);


//! \brief     Sets the stator resistance Q format in the estimator in 8 bit unsigned integer (uint_least8_t).
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rs_qFmt  The stator resistance Q format
extern void EST_setRs_qFmt(EST_Handle handle,uint_least8_t Rs_qFmt);


//! \brief     Updates the Id reference value used for online stator resistance estimation in per unit (pu), IQ24.
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pId_ref_pu  The pointer to the Id reference value, pu
extern void EST_updateId_ref_pu(EST_Handle handle,_iq *pId_ref_pu);


//! \brief      Updates the estimator state
//! \param[in]  handle        The estimator (EST) handle
//! \param[in]  Id_target_pu  The target Id current during each estimator state, pu IQ24
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool EST_updateState(EST_Handle handle,const _iq Id_target_pu);


//! \brief     Determines if a zero Iq current reference should be used in the controller
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if a zero Iq current reference should be used (true) or not (false)
extern bool EST_useZeroIq_ref(EST_Handle handle);


//! \brief     Computes the direction Q format for the estimator
//! \param[in] handle    The estimator (EST) handle
//! \param[in] flux_max  The maximum flux value
//! \return    The direction Q format
#ifdef __TMS320C28XX_FPU32__
extern uint_least8_t EST_computeDirection_qFmt(EST_Handle handle,const int32_t flux_max);
#else
extern uint_least8_t EST_computeDirection_qFmt(EST_Handle handle,const float_t flux_max);
#endif


//! \brief     Gets the direction Q format from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The direction Q format
extern uint_least8_t EST_getDir_qFmt(EST_Handle handle);


//! \brief     Sets the direction Q format in the estimator
//! \param[in] handle    The estimator (EST) handle
//! \param[in] dir_qFmt  The direction Q format
extern void EST_setDir_qFmt(EST_Handle handle,const uint_least8_t dir_qFmt);


//! \brief     Sets maximum negative electrical frequency from the estimator
//! \param[in] handle         The estimator (EST) handle
//! \param[in] fe_neg_max_pu  The maximum negative electrical frequency, Hz
extern void EST_setFe_neg_max_pu(EST_Handle handle,const _iq fe_neg_max_pu);


//! \brief     Sets minimum positive electrical frequency from the estimator
//! \param[in] handle          The estimator (EST) handle
//! \param[in] fe_pos_min_pu   The minimum positive electrical frequency, Hz
extern void EST_setFe_pos_min_pu(EST_Handle handle,const _iq fe_pos_min_pu);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_H_ definition

