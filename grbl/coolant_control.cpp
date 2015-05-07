/*
  coolant_control.c - coolant control methods
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
  */ 

#include "system.h"
#include "coolant_control.h"
#include "protocol.h"
#include "gcode.h"


void coolant_init()
{
////////////////////////####################******************
#if MotherBoard==3 ///**MB==3
	SET_OUTPUT(COOLANT_FLOOD_PIN);
#ifdef ENABLE_M7
	SET_OUTPUT(COOLANT_MIST_PIN);
#endif
#else  ///**MB!=3
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
  #endif
#endif ////**MB==3
////////////////////////####################******************
  coolant_stop();
}


void coolant_stop()
{
////////////////////////####################******************
#if MotherBoard==3 ///**MB==3
		WRITE(COOLANT_FLOOD_PIN,COOLANT_FLOOD_INVERTING);  ///****+logic  Low   -logic High
#ifdef ENABLE_M7
		WRITE(COOLANT_FLOOD_PIN,COOLANT_MIST_INVERTING);   ///****+logic  Low   -logic High
#endif
#else  ///**MB!=3
  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
  #endif
#endif ////**MB==3
////////////////////////####################******************
}



void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  protocol_auto_cycle_start();   //temp fix for M8 lockup
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.
  ////////////////////////####################******************
#if MotherBoard==3 ///**MB==3
	if (mode == COOLANT_FLOOD_ENABLE) {
	  WRITE(COOLANT_FLOOD_PIN,!COOLANT_FLOOD_INVERTING);  ///****+logic High   -logic Low
	  }
	else if (mode == COOLANT_MIST_ENABLE) {
#ifdef ENABLE_M7   /////***not the same mode..why here?
	  WRITE(COOLANT_FLOOD_PIN,!COOLANT_MIST_INVERTING);    ///****+logic High -logic Low
#endif
	  ;
	  }
#else  ///**MB!=3
	if (mode == COOLANT_FLOOD_ENABLE) {
	  COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT); 
	  } 
	else if (mode == COOLANT_MIST_ENABLE) {
  #ifdef ENABLE_M7 
		COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
  #endif
	;
	 }
#endif ////**MB==3
  ////////////////////////####################******************
	else {
	  coolant_stop();
	}

}
