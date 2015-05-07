/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  This file is part of PiBotGRBL-Firmware.

  PiBotGrbl-Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  At the same time you have to follow the rules of PiBot.

  PiBotGRBL-Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
   You should have received a copy of both the GNU General Public License 
   and PiBot License along with PiBot-Firmware.  
   If not, see <http://www.gnu.org/licenses/> and <http://www.pibot.com/>.

  PiBotGRBL-Firmware is based on the official GRBL. We change it for the goal
  of easy use and configuration. It compatible with both of orignal GRBL settings 
  and PiBot quick setting style.

   Main author: Simen Svale Skogsrud, Sungeun K. Jeon & Thomas Pan
   
   mainpage PiBotGRBL-Firmware for Arduino based GRBL
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2013 Sungeun K. Jeon
    Copyright (c) 2014 by Thomas
*/   
  
#include "system.h"
#include "settings.h"
#include "protocol.h"
#include "planner.h"
#include "stepper.h"
#include "motion_control.h"
#include "limits.h"
#include "report.h"

// Homing axis search distance multiplier. Computed by this value times the axis max travel.
#define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.


void limits_init() 
{
	///////////////////##########################
#if MotherBoard==3 && !defined(limit_int_style)  ///*** MB==3
#if defined(X_MIN_PIN) && X_MIN_PIN>-1   //////##############
		SET_INPUT(X_MIN_PIN);
#endif
#if defined(Y_MIN_PIN) && Y_MIN_PIN>-1   //////##############
		SET_INPUT(Y_MIN_PIN);
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN>-1   //////##############
		SET_INPUT(Z_MIN_PIN);
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN>-1   //////##############
		SET_INPUT(X_MAX_PIN);
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN>-1   //////##############
		SET_INPUT(Y_MAX_PIN);
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN>-1   //////##############
		SET_INPUT(Z_MAX_PIN);
#endif
	/*
#if(ENDSTOP_X_MIN_INVERTING)
	
#else
	
#endif*/
		if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) 
		{
			PULLUP(X_MIN_PIN,LOW);	 /////***** invert as machine SET LOW
			PULLUP(Y_MIN_PIN,LOW);
			PULLUP(Z_MIN_PIN,LOW);	
			PULLUP(X_MAX_PIN,LOW);	 /////***** invert as machine SET LOW
			PULLUP(Y_MAX_PIN,LOW);
			PULLUP(Z_MAX_PIN,LOW);
		} 
		else 
		{
			PULLUP(X_MIN_PIN,HIGH);   /////***** invert as machine SET HIGH
			PULLUP(Y_MIN_PIN,HIGH);
			PULLUP(Z_MIN_PIN,HIGH);
			PULLUP(X_MAX_PIN,HIGH);   /////***** invert as machine SET HIGH
			PULLUP(Y_MAX_PIN,HIGH);
			PULLUP(Z_MAX_PIN,HIGH);
		}	
		
		if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) { 	
		  LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt   ///****7=0x00000111
		  PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
		} else {
		  limits_disable(); 
		}
		
#else   ///*** MB!=3

  LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins

  if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) {
    LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Requires external pull-down.
  } else {
    LIMIT_PORT |= (LIMIT_MASK);  // Enable internal pull-up resistors. Normal high operation.
  }

  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
  } else {
    limits_disable(); 
  }
  
  #ifdef ENABLE_SOFTWARE_DEBOUNCE
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
  #endif
  
 #endif  ///*** MB==3
//////###############################
}


void limits_disable()
{
  LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
  PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
}


// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing 
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a 
// bouncing pin without a debouncing method. A simple software debouncing feature may be enabled 
// through the config.h file, where an extra timer delays the limit pin read by several milli-
// seconds to help with, not fix, bouncing switches.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
#ifndef ENABLE_SOFTWARE_DEBOUNCE
  ISR(LIMIT_INT_vect) // DEFAULT: Limit pin change interrupt process. 
  {
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
    if (sys.state != STATE_ALARM) { 
      if (bit_isfalse(sys.execute,EXEC_ALARM)) {
        mc_reset(); // Initiate system kill.
        bit_true_atomic(sys.execute, (EXEC_ALARM | EXEC_CRIT_EVENT)); // Indicate hard limit critical event
      }
    }
  }  
#else // OPTIONAL: Software debounce limit pin routine.
  // Upon limit pin change, enable watchdog timer to create a short delay. 
  ISR(LIMIT_INT_vect) { if (!(WDTCSR & (1<<WDIE))) { WDTCSR |= (1<<WDIE); } }
  ISR(WDT_vect) // Watchdog timer ISR
  {
    WDTCSR &= ~(1<<WDIE); // Disable watchdog timer. 
    if (sys.state != STATE_ALARM) {  // Ignore if already in alarm state. 
      if (bit_isfalse(sys.execute,EXEC_ALARM)) {
		  ////////////////////////////////////////////////#####################################
#if MotherBoard==3 && !defined(limit_int_style) /////**MB==3   /////***this not work
			  uint8_t bits = 0;
		  //************Get Limit Pin State.  			////*********add MAX get
#if LIMIT_MAX_OPEN   ////*******max control,   Let Max do as Min
			  if(isXMinEndstopHit()||isXMaxEndstopHit()) { bits|= MASK(X_MASK);} 
			  if(isYMinEndstopHit()||isYMaxEndstopHit()) { bits|= MASK(Y_MASK);} 
			  if(isZMinEndstopHit()||isZMaxEndstopHit()) { bits|= MASK(Z_MASK);} 
#else
			  if(isXMinEndstopHit()) { bits|= MASK(X_MASK);} 
			  if(isYMinEndstopHit()) { bits|= MASK(Y_MASK);} 
			  if(isZMinEndstopHit()) { bits|= MASK(Z_MASK);} 
#endif
#else  /////**MB!=3   ////////#################################################
		  //************Get Limit Pin State.
				uint8_t bits = LIMIT_PIN;
#endif   /////**MB==3
		  //////////////////////////////////////////////########################################

        // Check limit pin state. 
#if LIMIT_MAX_OPEN   ////*******max control,   Let Max do as Min
        if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { bits ^= LIMIT_MIN_MASK; }
        if (bits & LIMIT_MIN_MASK) {
          mc_reset(); // Initiate system kill.
          bit_true_atomic(sys.execute, (EXEC_ALARM | EXEC_CRIT_EVENT)); // Indicate hard limit critical event
        }
#else
        if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { bits ^= LIMIT_MASK; }
        if (bits & LIMIT_MASK) {
          mc_reset(); // Initiate system kill.
          bit_true_atomic(sys.execute, (EXEC_ALARM | EXEC_CRIT_EVENT)); // Indicate hard limit critical event
        }
#endif
      }  
    }
  }
#endif


// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock 
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically 
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort runtime command can interrupt this process.
void limits_go_home(uint8_t cycle_mask) 
{
  if (sys.abort) { return; } // Block if system reset has been issued.

  // Initialize homing in search mode to quickly engage the specified cycle_mask limit switches.
  bool approach = true;
  float homing_rate = settings.homing_seek_rate;
  uint8_t invert_pin, idx;
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);  ///***5
  float target[N_AXIS];
  
  uint8_t limit_pin[N_AXIS], step_pin[N_AXIS];
  float max_travel = 0.0;
  for (idx=0; idx<N_AXIS; idx++) {  
    // Initialize limit and step pin masks
    limit_pin[idx] = get_limit_pin_mask(idx);    ///////****get the Pin  of  limit min x, y ,z ,  Limit OK
    step_pin[idx] = get_step_pin_mask(idx);       ///////****get the Pin  of  limit min x, y ,z   now I let it 0, 1, 2

    // Determine travel distance to the furthest homing switch based on user max travel settings.
    // NOTE: settings.max_travel[] is stored as a negative value.
    if (max_travel > settings.max_travel[idx]) { max_travel = settings.max_travel[idx]; }
  }
  max_travel *= -HOMING_AXIS_SEARCH_SCALAR; // Ensure homing switches engaged by over-estimating max travel.
  
  plan_reset(); // Reset planner buffer to zero planner current position and to clear previous motions.
  
  do {
    // Initialize invert_pin boolean based on approach and invert pin user setting.
    if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { invert_pin = approach; }
    else { invert_pin = !approach; }

    // Initialize and declare variables needed for homing routine.
    uint8_t n_active_axis = 0;
    uint8_t axislock = 0;
        
    for (idx=0; idx<N_AXIS; idx++) {
      // Set target location for active axes and setup computation for homing rate.
      if (bit_istrue(cycle_mask,bit(idx))) { 
        n_active_axis++;
        if (!approach) { target[idx] = -max_travel; }
        else { target[idx] = max_travel; }
      } else {
        target[idx] = 0.0;
      }


      // Set target direction based on cycle mask
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) { target[idx] = -target[idx]; }
      
      // Apply axislock to the step port pins active in this cycle.
      if (bit_istrue(cycle_mask,bit(idx))) { axislock |= step_pin[idx]; }
    }      
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.
    sys.homing_axis_lock = axislock;
  
    // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
    uint8_t limit_state;
    
    #ifdef USE_LINE_NUMBERS
      plan_buffer_line(target, homing_rate, false, HOMING_CYCLE_LINE_NUMBER); // Bypass mc_line(). Directly plan homing motion.
    #else
      plan_buffer_line(target, homing_rate, false); // Bypass mc_line(). Directly plan homing motion.
    #endif
    
    st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
    st_wake_up(); // Initiate motion
    do {
      // Check limit state. Lock out cycle axes when they change.
	  /////////////////########################################***********
#if MotherBoard==3 /////**MB==3  
#ifdef limit_int_style
		  limit_state = LIMIT_PIN;
	      if (invert_pin) { limit_state ^= LIMIT_MASK; }
#else
	  //************Get Limit Pin State.	 PiBot get limit/endstop mask same with the step mask.
#if LIMIT_MAX_OPEN
		  if(isXMinEndstopHit()) { limit_state^= MASK(X_LIMIT_BIT);}   ////***read X_endstop then write into limite_state
		  if(isYMinEndstopHit()) { limit_state^= MASK(Y_LIMIT_BIT);} 
		  if(isZMinEndstopHit()) { limit_state^= MASK(Z_LIMIT_BIT);}   ////*****add MAX limit
		  if(isXMaxEndstopHit()) { limit_state^= MASK(X_LIMIT_MAX_BIT);}   ////***read X_endstop then write into limite_state
		  if(isYMaxEndstopHit()) { limit_state^= MASK(Y_LIMIT_MAX_BIT);} 
		  if(isZMaxEndstopHit()) { limit_state^= MASK(Z_LIMIT_MAX_BIT);}   ////*****add MAX limit
#else
		  if(isXMinEndstopHit()) { limit_state^= MASK(X_MASK);} ////***read X_endstop then write into limite_state
		  if(isYMinEndstopHit()) { limit_state^= MASK(Y_MASK);} 
		  if(isZMinEndstopHit()) { limit_state^= MASK(Z_MASK);} ////*****add MAX limit
#endif
#endif
#else  /////**MB!=3   ////////#################################################
      limit_state = LIMIT_PIN;
      if (invert_pin) { limit_state ^= LIMIT_MASK; }
#endif   /////**MB==3
	  /////////////////////////////#####################################

      for (idx=0; idx<N_AXIS; idx++) {
        if (axislock & step_pin[idx]) {
          if (limit_state & limit_pin[idx]) { axislock &= ~(step_pin[idx]); }
        }
      }
      sys.homing_axis_lock = axislock;
      st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.
      // Check only for user reset. No time to run protocol_execute_runtime() in this loop.
      if (sys.execute & EXEC_RESET) { protocol_execute_runtime(); return; }
    } while (STEP_MASK & axislock);
    
    st_reset(); // Immediately force kill steppers and reset step segment buffer.
    plan_reset(); // Reset planner buffer. Zero planner positions. Ensure homing motion is cleared.

    delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.

    // Reverse direction and reset homing rate for locate cycle(s).
    homing_rate = settings.homing_feed_rate;
    approach = !approach;
    
  } while (n_cycle-- > 0);
    
  // The active cycle axes should now be homed and machine limits have been located. By 
  // default, grbl defines machine space as all negative, as do most CNCs. Since limit switches
  // can be on either side of an axes, check and set axes machine zero appropriately. Also,
  // set up pull-off maneuver from axes limit switches that have been homed. This provides
  // some initial clearance off the switches and should also help prevent them from falsely
  // triggering when hard limits are enabled or when more than one axes shares a limit pin.
  for (idx=0; idx<N_AXIS; idx++) {
    // Set up pull off targets and machine positions for limit switches homed in the negative
    // direction, rather than the traditional positive. Leave non-homed positions as zero and
    // do not move them.
    // NOTE: settings.max_travel[] is stored as a negative value.
    if (cycle_mask & bit(idx)) {
    
      #ifdef HOMING_FORCE_SET_ORIGIN
        sys.position[idx] = 0;  // Set axis homed location as axis origin
        target[idx] = settings.homing_pulloff;  
        if ( bit_isfalse(settings.homing_dir_mask,bit(idx)) ) { target[idx] = -target[idx]; }     
      #else
        if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
          target[idx] = settings.homing_pulloff+settings.max_travel[idx];
          sys.position[idx] = lround(settings.max_travel[idx]*settings.steps_per_mm[idx]);
        } else {
          target[idx] = -settings.homing_pulloff;
          sys.position[idx] = 0;
        }
      #endif
      
    } else { // Non-active cycle axis. Set target to not move during pull-off. 
      target[idx] = (float)sys.position[idx]/settings.steps_per_mm[idx];
    }
  }
  plan_sync_position(); // Sync planner position to current machine position for pull-off move.
  
  #ifdef USE_LINE_NUMBERS
    plan_buffer_line(target, settings.homing_seek_rate, false, HOMING_CYCLE_LINE_NUMBER); // Bypass mc_line(). Directly plan motion.
  #else
    plan_buffer_line(target, settings.homing_seek_rate, false); // Bypass mc_line(). Directly plan motion.
  #endif
  
  // Initiate pull-off using main motion control routines. 
  // TODO : Clean up state routines so that this motion still shows homing state.
  sys.state = STATE_QUEUED;
  bit_true_atomic(sys.execute, EXEC_CYCLE_START);
  protocol_execute_runtime();
  protocol_buffer_synchronize(); // Complete pull-off motion.
  
  // Set system state to homing before returning. 
  sys.state = STATE_HOMING; 
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
void limits_soft_check(float *target)
{
  uint8_t idx;
  uint8_t soft_limit_error = false;
  for (idx=0; idx<N_AXIS; idx++) {
   
    #ifdef HOMING_FORCE_SET_ORIGIN
      // When homing forced set origin is enabled, soft limits checks need to account for directionality.
      // NOTE: max_travel is stored as negative
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { soft_limit_error = true; }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { soft_limit_error = true; }
      }
    #else  
      // NOTE: max_travel is stored as negative
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { soft_limit_error = true; }
    #endif
    
    if (soft_limit_error) {
      // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within 
      // workspace volume so just come to a controlled stop so position is not lost. When complete
      // enter alarm mode.
      if (sys.state == STATE_CYCLE) {
        bit_true_atomic(sys.execute, EXEC_FEED_HOLD);
        do {
          protocol_execute_runtime();
          if (sys.abort) { return; }
        } while ( sys.state != STATE_IDLE || sys.state != STATE_QUEUED);
      }
    
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      bit_true_atomic(sys.execute, (EXEC_ALARM | EXEC_CRIT_EVENT)); // Indicate soft limit critical event
      protocol_execute_runtime(); // Execute to enter critical event loop and system abort
      return;
    }
  }
}


#if MotherBoard==3
/////###############PiBot endstop check 
/////////####################################********thomas
    static inline bool isXMinEndstopHit()
    {
#if defined(X_MIN_PIN) && X_MIN_PIN>-1  
        return READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isYMinEndstopHit()
    {
#if defined(Y_MIN_PIN) && Y_MIN_PIN>-1  
        return READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isZMinEndstopHit()
    {
#if defined(Z_MIN_PIN) && Z_MIN_PIN>-1  
        return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
#else
        return false;
#endif
    }

///////*****add MAX check function
    static inline bool isXMaxEndstopHit()
    {
#if defined(X_MAX_PIN) && X_MAX_PIN>-1  
        return READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isYMaxEndstopHit()
    {
#if defined(Y_MAX_PIN) && Y_MAX_PIN>-1  
        return READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isZMaxEndstopHit()
    {
#if defined(Z_MAX_PIN) && Z_MAX_PIN>-1  
        return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
#else
        return false;
#endif
    }
///////////////////////////######################*******
#endif