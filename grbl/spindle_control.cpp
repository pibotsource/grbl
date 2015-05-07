/*
  spindle_control.c - spindle control methods
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
	  Copyright (c) 2009-2011 Simen Svale Skogsrud
	  Copyright (c) 2011-2013 Sungeun K. Jeon
	  Copyright (c) 2014 by Thomas
  */  


#include "system.h"
#include "spindle_control.h"
#include "protocol.h"
#include "gcode.h"


void spindle_init()
{    
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  /////////////////#################################**************************			
#if MotherBoard==3  ////***MB==3
#ifdef VARIABLE_SPINDLE
	  SET_OUTPUT(SPINDLE_PWM_PIN);		// Configure as PWM output pin. 
#ifndef CPU_MAP_ATMEGA328P   ///***MCU2560 or other more source
#if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN>-1
	  SET_OUTPUT(SPINDLE_ENABLE_PIN);	// Configure as PWM output pin.
#endif
#endif
#else	 //VARIABLE_SPINDLE
#if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN>-1
	  SET_OUTPUT(SPINDLE_ENABLE_PIN);	// Configure as PWM output pin.
#endif
#endif
#if defined(SPINDLE_DIR_PIN) && SPINDLE_DIR_PIN>-1
	  SET_OUTPUT(SPINDLE_DIR_PIN);	  // Configure as PWM output pin.
#endif
#else   ////***MB!=3
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.  
#ifdef VARIABLE_SPINDLE    
	  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.	  
#ifndef CPU_MAP_ATMEGA328P		
	  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.	
#endif	   
#else	
	  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.  
#endif	
	  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
#endif ////***MB==3
  /////////////////#################################**************************		

  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
/////////////////#################################**************************		  
#if MotherBoard==3  ////***MB==3
#if VARIABLE_SPINDLE  ///** timer2A work 
  TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
  #ifndef CPU_MAP_ATMEGA328P 
#if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN>-1   
	WRITE(SPINDLE_ENABLE_PIN,(SPINDLE_ENABLE_INVERTING? HIGH : LOW));      // Set pin to low.
#endif	
  #endif
#else
#if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN>-1
	WRITE(SPINDLE_ENABLE_PIN,(SPINDLE_ENABLE_INVERTING? HIGH : LOW));      // Set pin to low.
#endif	
#endif	
#else   ////***MB!=3
#if VARIABLE_SPINDLE
  TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
  #ifndef CPU_MAP_ATMEGA328P 
   SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.		
  #endif
#else
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
#endif	
#endif ////***MB==3
 /////////////////#################################**************************	  
}


void spindle_run(uint8_t direction, float rpm) 
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  
  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_auto_cycle_start();  //temp fix for M3 lockup
  protocol_buffer_synchronize(); 

  // Halt or set spindle direction and rpm. 
  if (direction == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

	/////////////////#################################**************************			  
#if MotherBoard==3  ////***MB==3
		if (direction == SPINDLE_ENABLE_CW) {
			WRITE(SPINDLE_DIR_PIN,LOW);
		} else {
			WRITE(SPINDLE_DIR_PIN,HIGH);
		}
	
#else
		if (direction == SPINDLE_ENABLE_CW) {
		  SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
		} else {
		  SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
		}
#endif

    #ifdef VARIABLE_SPINDLE
      // TODO: Install the optional capability for frequency-based output for servos.
      #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
      TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
      rpm -= SPINDLE_MIN_RPM;
      if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent uint8 overflow
      uint8_t current_pwm = floor( rpm*(255.0/SPINDLE_RPM_RANGE) + 0.5);
      OCR_REGISTER = current_pwm;
    
      #ifndef CPU_MAP_ATMEGA328P // On the Uno, spindle enable and PWM are shared.
	/////////////////#################################**************************			
#if MotherBoard==3  ////***MB==3
		WRITE(SPINDLE_ENABLE_PIN,(SPINDLE_ENABLE_INVERTING? LOW: HIGH));
#else
			SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
#endif
	/////////////////##################*********************************************
      #endif
    #else   
	/////////////////#################################**************************			
#if MotherBoard==3  ////***MB==3
		WRITE(SPINDLE_ENABLE_PIN,(SPINDLE_ENABLE_INVERTING? LOW: HIGH));
#else
		  SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
#endif
	/////////////////##################*********************************************	
    #endif

  }
}
