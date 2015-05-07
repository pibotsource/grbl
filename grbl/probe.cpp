/*
  probe.c - code pertaining to probing methods
	This file is part of PiBot-Firmware.
  
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
	 Copyright (c) 2014 Sungeun K. Jeon
	 Copyright (c) 2014 by Thomas
  */  

  
#include "system.h"
#include "settings.h"
#include "probe.h"

// Inverts the probe pin state depending on user settings.
uint8_t probe_invert_mask;


// Probe pin initialization routine.
void probe_init() 
{
#if MotherBoard==3
	SET_INPUT(PI_PROBE_PIN);
	if (bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { 
//    PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down.
    probe_invert_mask = 0;
  } else {
   	PULLUP(PI_PROBE_PIN,HIGH);    // Enable internal pull-up resistors. Normal high operation.
    probe_invert_mask = PROBE_MASK; 
  }

#else
  PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins
  if (bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { 
    PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down.
    probe_invert_mask = 0;
  } else {
    PROBE_PORT |= PROBE_MASK;    // Enable internal pull-up resistors. Normal high operation.
    probe_invert_mask = PROBE_MASK; 
  }
#endif
}


// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state() 
{ 
#if MotherBoard==3
	return (READ(PI_PROBE_PIN)!=PROBE_INVERTING);	
#else
	return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); 
#endif
}


// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor()
{
  if (sys.probe_state == PROBE_ACTIVE) { 
    if (probe_get_state()) {
      sys.probe_state = PROBE_OFF;
      memcpy(sys.probe_position, sys.position, sizeof(float)*N_AXIS);
      bit_true(sys.execute, EXEC_FEED_HOLD);
    }
  }
}
