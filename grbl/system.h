/*
  system.h - Header for system level commands and real-time processes
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


#ifndef system_h
#define system_h

// Define system header files and standard libraries used by Grbl
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
//////********
#ifndef false 
#define false 0
#endif
#ifndef true
#define true 1
#endif

#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#include "fastio.h"
//////////////////////*********************####################
#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value. Must start with 0 and be continuous.
#define Y_AXIS 1
#define Z_AXIS 2

// Define Grbl configuration and shared header files
#include "config.h"
#include "defaults.h"
#include "nuts_bolts.h"

///////###################****
#ifndef MotherBoard
#define MotherBoard 3    ///Default pibot 
#endif

#if MotherBoard==3///////***for pibot pins.h to system.h   ///***I set the mask fixed
//////###################********* 
#if N_AXIS<4 
#define X_MASK   0
#define Y_MASK   1
#define Z_MASK   2
#define STEP_MASK       ((1<<X_MASK)|(1<<Y_MASK)|(1<<Z_MASK))
#define DIRECTION_MASK  ((1<<X_MASK)|(1<<Y_MASK)|(1<<Z_MASK))
#else 
#define X_MASK	 0
#define Y_MASK	 1
#define Z_MASK	 2
#define T_MASK	 3
#define STEP_MASK       ((1<<X_MASK)|(1<<Y_MASK)|(1<<Z_MASK)|(1<<T_MASK))
#define DIRECTION_MASK  ((1<<X_MASK)|(1<<Y_MASK)|(1<<Z_MASK)|(1<<T_MASK))
#endif

#ifdef limit_int_style
// NOTE: All limit bit pins must be on the same port
#if X_MIN_PIN>=62 && X_MIN_PIN<=69
#define LIMIT_DDR		DDRK		 ///****B	
#define LIMIT_PORT		PORTK
#define LIMIT_PIN		PINK
#define X_LIMIT_BIT 	(X_MIN_PIN-62)      // MEGA2560 Digital Pin 62       0
#define Y_LIMIT_BIT 	(Y_MIN_PIN-62)      // MEGA2560 Digital Pin 64       2
#define Z_LIMIT_BIT 	(Z_MIN_PIN-62)      // MEGA2560 Digital Pin 66       4  
#if LIMIT_MAX_OPEN
///////********* add Max int IO port,  only work on PiBot Rev2.0 
#define X_LIMIT_MAX_BIT 	(X_MAX_PIN-62)      // MEGA2560 Digital Pin 62       0
#define Y_LIMIT_MAX_BIT 	(Y_MAX_PIN-62)      // MEGA2560 Digital Pin 64       2
#define Z_LIMIT_MAX_BIT 	(Z_MAX_PIN-62)      // MEGA2560 Digital Pin 66       4  
#endif
#define LIMIT_INT		PCIE2        // Pin change interrupt enable pin
#define LIMIT_INT_vect	PCINT2_vect 
#define LIMIT_PCMSK 	PCMSK2       // Pin change interrupt register
#endif

#if X_MIN_PIN>=10 && X_MIN_PIN<=13          /////****  50-53 for ISP Port use
#define LIMIT_DDR		DDRB		 ///****B	
#define LIMIT_PORT		PORTB
#define LIMIT_PIN		PINB
#define X_LIMIT_BIT 	(X_MIN_PIN-6)      // MEGA2560 Digital Pin    10-13     4-7
#define Y_LIMIT_BIT 	(Y_MIN_PIN-6)      // MEGA2560 Digital Pin        
#define Z_LIMIT_BIT 	(Z_MIN_PIN-6)      // MEGA2560 Digital Pin     
#define LIMIT_INT		PCIE0        // Pin change interrupt enable pin
#define LIMIT_INT_vect	PCINT0_vect 
#define LIMIT_PCMSK 	PCMSK0       // Pin change interrupt register
#endif
//////*******************************************
////******pins.h key input interrrupt use/// PCINT 0 2 4  ; Digtal 10 11 12   PORT: PB  
/*
#define LIMIT_INT		 PCIE0	// Pin change interrupt enable pin
#define LIMIT_INT_vect	 PCINT0_vect 
#define LIMIT_PCMSK 	 PCMSK0 // Pin change interrupt register    
*/
////////********add max IO Port 
#if LIMIT_MAX_OPEN
#define LIMIT_MIN_MASK   ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT))
#define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<X_LIMIT_MAX_BIT)|(1<<Y_LIMIT_MAX_BIT)|(1<<Z_LIMIT_MAX_BIT)) // All limit bits
#else
#define LIMIT_MASK   ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT))
#endif
#else
#define LIMIT_MASK		 ((1<<X_MASK)|(1<<Y_MASK)|(1<<Z_MASK)) // All limit bits   ///*****X_LIMIT_BIT   Y_LIMIT_BIT   Z_LIMIT_BIT
#endif

#define PROBE_MASK  (1<<(PI_PROBE_PIN-6))
/////#########################################################******************
/////******fast set the uint8_t data into one pin operation (bool)
#define U8_PIN(Va,Ma)  ((Va & MASK(Ma))>0 ? HIGH : LOW)  
///////****************######################## Thomas
#define bit_clear(x,y) x&= ~(1<<y)  //cbi(x,y)
#define bit_set(x,y)   x|= (1<<y)   //sbi(x,y)
//////*****************########################
///////////////////////*******######## Timer0B   OC0B :PW4 interrupte service routine: ISR(TIMER0_COMPB_vect)
/*#define PWM_TIMER_VECTOR TIMER0_COMPB_vect   //eg
#define PWM_OCR OCR0B
#define PWM_TCCR TCCR0A
#define PWM_TIMSK TIMSK0
#define PWM_OCIE OCIE0B    */
////******pins.h key input interrrupt use/// PCINT 16 17 18 19 20 21 22 23   PORT: Pk   analogPIN
#if RESET_INPUT_PIN >=62 && RESET_INPUT_PIN<=69
#define PINOUT_INT		 PCIE2	// Pin change interrupt enable pin
#define PINOUT_INT_vect  PCINT2_vect
#define PINOUT_PCMSK	 PCMSK2 // Pin change interrupt register
#endif
#if RESET_INPUT_PIN>=10 && RESET_INPUT_PIN<=13  
#define PINOUT_INT		 PCIE0	// Pin change interrupt enable pin
#define PINOUT_INT_vect  PCINT0_vect
#define PINOUT_PCMSK	 PCMSK0 // Pin change interrupt register
#endif


#if VARIABLE_SPINDLE   /////****Pins.h to 
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER2B which is attached to Digital Pin 9  PWM9
  #define TCCRA_REGISTER	  TCCR2A
  #define TCCRB_REGISTER	  TCCR2B
  #define OCR_REGISTER		  OCR2B

  #define COMB_BIT			  COM2B1
  #define WAVE0_REGISTER	  WGM20
  #define WAVE1_REGISTER	  WGM21
  #define WAVE2_REGISTER	  WGM22
  #define WAVE3_REGISTER	  WGM23
#endif

/////////****************###########I added this bool for serial connection start flag
 static bool Serial_begin =false;	 
 static bool  Serial_connected =false; 
//////////////////////////////###############************************
#endif   //////***end MB==3

#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)

#define TICKS_PER_MICROSECOND (F_CPU/1000000)

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))


// Define system executor bit map. Used internally by runtime protocol as runtime command flags, 
// which notifies the main program to execute the specified runtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the runtime protocol only needs to check for a non-zero value to 
// know when there is a runtime command to execute.
#define EXEC_STATUS_REPORT  bit(0) // bitmask 00000001
#define EXEC_CYCLE_START    bit(1) // bitmask 00000010
#define EXEC_CYCLE_STOP     bit(2) // bitmask 00000100
#define EXEC_FEED_HOLD      bit(3) // bitmask 00001000
#define EXEC_RESET          bit(4) // bitmask 00010000
#define EXEC_ALARM          bit(5) // bitmask 00100000
#define EXEC_CRIT_EVENT     bit(6) // bitmask 01000000
// #define                  bit(7) // bitmask 10000000

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE       0      // Must be zero. No flags.
#define STATE_ALARM      bit(0) // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE bit(1) // G-code check mode. Locks out planner and motion only.
#define STATE_HOMING     bit(2) // Performing homing cycle
#define STATE_QUEUED     bit(3) // Indicates buffered blocks, awaiting cycle start.
#define STATE_CYCLE      bit(4) // Cycle is running
#define STATE_HOLD       bit(5) // Executing feed hold
// #define STATE_JOG     bit(6) // Jogging mode is unique like homing.


// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.
  uint8_t state;                 // Tracks the current state of Grbl.
  volatile uint8_t execute;      // Global system runtime executor bitflag variable. See EXEC bitmasks.
  uint8_t homing_axis_lock;
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps. 
                                 // NOTE: This may need to be a volatile variable, if problems arise.                             
  uint8_t auto_start;            // Planner auto-start flag. Toggled off during feed hold. Defaulted by settings.
  volatile uint8_t probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
  int32_t probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
} system_t;
extern system_t sys;


// Initialize the serial protocol
void system_init();

// Executes an internal system command, defined as a string starting with a '$'
uint8_t system_execute_line(char *line);

// Checks and executes a runtime command at various stop points in main program
void system_execute_runtime();

// Execute the startup script lines stored in EEPROM upon initialization
void system_execute_startup(char *line);

#endif
