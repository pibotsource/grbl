/*
  probe.h - code pertaining to probing methods
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

  
#ifndef probe_h
#define probe_h 

// Values that define the probing state machine.  
#define PROBE_OFF     0 // No probing. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.


// Probe pin initialization routine.
void probe_init();

// Returns probe pin state.
uint8_t probe_get_state();

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void probe_state_monitor();

#endif
