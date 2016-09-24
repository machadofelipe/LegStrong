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
//! \file   modules/enc/src/32b/enc.c
//! \brief  Portable C fixed point code.  These functions define the
//! \brief  encoder routines.
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "enc.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals

// **************************************************************************
// the functions

ENC_Handle ENC_init(void *pMemory,const size_t numBytes)
{
  ENC_Handle encHandle;
  
  if(numBytes < sizeof(ENC_Obj))
    return((ENC_Handle)NULL);

  // assign the handle
  encHandle = (ENC_Handle)pMemory;
  
  return(encHandle);
} // end of ENC_init() function

void ENC_setup(ENC_Handle encHandle, const int16_t sample_period, const uint16_t num_pole_pairs, const uint16_t num_enc_slots, const uint32_t enc_zero_offset, const float_t full_scale_freq, const float_t speed_update_freq, const float_t speed_cutoff)
{
  ENC_Obj *enc;
  float_t temp;
  float_t speed_cutoff_radians;
  int16_t i;
 
  // create an object pointer for manipulation
  enc = (ENC_Obj *)encHandle;
  
  // setup the encoder sample rate
  enc->sample_count = 0;
  enc->sample_period = sample_period;
  
  // copy the parameters into the data structure
  enc->num_enc_slots = num_enc_slots;
  enc->num_pole_pairs = num_pole_pairs;
  enc->enc_zero_offset = enc_zero_offset;
  enc->full_scale_freq = full_scale_freq;
  enc->delta_enc = 0;
  enc->prev_enc = 0;
  
  // initialize the electrical angle
  enc->enc_elec_angle = 0;
    
  // compute the gain which translates the mech into the elec angle
  enc->mech_angle_gain = (_iq)((((uint32_t)1)<<24)/(4*num_enc_slots));

  // compute the speed gain
  temp = ((float_t)num_pole_pairs*speed_update_freq*ENC_SPEED_SCALING_FACTOR) / (4.0*(float_t)num_enc_slots*full_scale_freq*(float_t)sample_period);
  enc->speed_gain = (int32_t) temp;
  
  // compute the rpm gain
  temp = (float_t)((full_scale_freq*60.0)/num_pole_pairs);
  enc->rpm_gain = (int32_t) temp;
  
  // compute the speed coefficients and initialize the low-pass filtered output
  enc->speed_cutoff = speed_cutoff;
  speed_cutoff_radians = ENC_2PI*speed_cutoff;
  temp = speed_cutoff_radians/(speed_update_freq+speed_cutoff_radians);
  enc->speed_lpf_cx = (int32_t) (ENC_SPEED_COEFF_SCALING*temp);
  enc->speed_lpf_cy = ENC_SPEED_COEFF_SCALING - enc->speed_lpf_cx;
  enc->speed_lpf_out = 0;
  
  // setup the encoder log
  enc->log_state        = ENC_LOG_STATE_IDLE;
  enc->run_flag         = 0;
  enc->trigger_idx      = 0;
  enc->trigger_delta    = 0;  
  enc->log_idx          = 0;
  enc->post_trigger_len = ENC_LOG_LEN>>1;
  enc->post_trigger_cnt = enc->post_trigger_len;
  for (i=0; i<ENC_LOG_LEN; i++)
  {
  	enc->log[i] = 0;
  }
  
  return;
} // end of ENC setup function


void ENC_calcElecAngle(ENC_Handle encHandle, uint32_t posnCounts)
{
	ENC_Obj *enc;
	uint32_t temp;
 
  // create an object pointer for manipulation
  enc = (ENC_Obj *) encHandle;
	
  // compute the mechanical angle
  temp = posnCounts*enc->mech_angle_gain;
  // add in calibrated offset
  temp += enc->enc_zero_offset;
  // convert to electrical angle
  temp = temp * enc->num_pole_pairs;
  // wrap around 1.0 (Q24)
  temp &= ((uint32_t) 0x00ffffff);
  // store encoder electrical angle
  enc->enc_elec_angle = (_iq)temp;
  // update the slip angle
  enc->enc_slip_angle = enc->enc_slip_angle + enc->incremental_slip;
  // wrap around 1.0 (Q24)
  enc->enc_slip_angle &= ((uint32_t) 0x00ffffff);
  // add in compensation for slip
  temp = temp + enc->enc_slip_angle;
  // wrap around 1.0 (Q24)
  temp &= ((uint32_t) 0x00ffffff);
  // store encoder magnetic angle
  enc->enc_magnetic_angle = (_iq)temp;
  
  return;
} // end of ENC_calc_elec_angle() function



void ENC_run(ENC_Handle encHandle, uint32_t posnCounts, uint16_t indextFlag, uint16_t dirFlag, int16_t log_flag)
{
	uint16_t dir;
	uint16_t index_event_before;
	uint16_t index_event_after;
	ENC_Obj *enc;
	int32_t enc_val;
	int32_t delta_enc;
	int32_t temp;
	int16_t sample_count;
 
  // create an object pointer for manipulation
  enc = (ENC_Obj *) encHandle;
  
  // update the encoder counter
  sample_count = enc->sample_count;
  sample_count++;
  
  // if it reaches the sample period, read and process encoder data
  if (sample_count == enc->sample_period)
  {
    enc->sample_count = 0;
    
    // check for index event before the encoder reading
    index_event_before = indextFlag;
  	
    // read the encoder
    enc_val = posnCounts;
	
    // compute the mechanical angle
    // compute the mechanical angle
    temp = (enc_val)*enc->mech_angle_gain;
    // add in calibrated offset
    temp += enc->enc_zero_offset;
    // convert to electrical angle
    temp = temp * enc->num_pole_pairs;
    // wrap around 1.0
    temp &= ((uint32_t) 0x00ffffff);

    enc->enc_elec_angle = (_iq)temp;
  
    /*********************/
    /* compute the SPEED */
    /*********************/
  
    // read QEP direction from quadrature direction latch flag
    dir = dirFlag;
  
    // check for index event after the encoder reading
    index_event_after = indextFlag;
  
    // handle a rollover event
    if (index_event_after)
    {
  	   if (dir)
  	   {
  	     delta_enc = enc_val + (4*enc->num_enc_slots-1) - enc->prev_enc;
  	   }
  	   else
  	   {
         delta_enc = enc->prev_enc + (4*enc->num_enc_slots-1) - enc_val;
  	   }
    }
    else
    {
       delta_enc = enc_val - enc->prev_enc;
    }
    
    // save off the delta encoder value in the data structure only if the index event before and after are the same
    if ((index_event_after == index_event_before) && (abs(delta_enc) < enc->num_enc_slots))
    	enc->delta_enc = delta_enc;
    
    // log the startup data
    switch(enc->log_state)
    {
    	// wait for run flag to be set
    	case ENC_LOG_STATE_IDLE:
    	{
	    	if (enc->run_flag)
	    	{
	    		enc->run_flag  = 0;
	    		enc->log_idx   = 0;
	    		enc->log_state = ENC_LOG_STATE_FREERUN;
	    	}
    	}
    	break;
    	
    	// collect data round robin until there's a trigger event
    	case ENC_LOG_STATE_FREERUN:
    	{
			enc->log[enc->log_idx] = (int16_t)enc_val;
			enc->log_idx = enc->log_idx + 1;
			if (enc->log_idx > ENC_LOG_LEN)
				enc->log_idx = 0;
			if (abs(delta_enc)>ENC_LOG_DELTA_TRIGGER_THRES)
			{
				enc->trigger_delta = delta_enc;
				enc->trigger_idx = enc->log_idx;
				enc->post_trigger_cnt = enc->post_trigger_len;
				enc->log_state = ENC_LOG_STATE_ACQUIRE;
			}
    	}    		
    	break;
    	
    	// when trigger occurs collect 1/2 a buffer of post-trigger information
    	case ENC_LOG_STATE_ACQUIRE:
    	{
			enc->log[enc->log_idx] = (int16_t)enc_val;
			enc->log_idx = enc->log_idx + 1;
			if (enc->log_idx > ENC_LOG_LEN)
				enc->log_idx = 0;
			enc->post_trigger_cnt = enc->post_trigger_cnt - 1;
			if (enc->post_trigger_cnt == 0)
				enc->log_state = ENC_LOG_STATE_IDLE;
    	}      	
    	break;
    }

    // shift it to Q24
    temp = enc->delta_enc*enc->speed_gain;
  
    // LPF the encoder
    temp = (enc->speed_lpf_cy)*enc->speed_lpf_out + (enc->speed_lpf_cx)*temp;
    temp >>= ENC_SPEED_COEFF_Q;
    enc->speed_lpf_out = temp;

    // copy the current into the previous value  
    enc->prev_enc = enc_val;
  }
  else
  {
  	enc->sample_count = sample_count;
  }
  
} // end of ENC_run() function


int16_t ENC_getSpeedRPM(ENC_Handle encHandle)
{
	ENC_Obj *enc;
	_iq temp;
 
  // create an object pointer for manipulation
  enc = (ENC_Obj *) encHandle;
  
  temp = (enc->speed_lpf_out >> ENC_RPM_Q1);
  temp = (temp * enc->rpm_gain) >> ENC_RPM_Q2;
	
  return (int16_t) temp;
} // end of ENC_getSpeedRPM() function


// end of file
