/*
  Pins.h
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
	 Copyright (c) 2014 by Thomas
  */  
/*  this part have been use an very easy style to config firmware, and you can use old board only by setting board style in config.h */

#ifndef PINS_H
#define PINS_H

//----------------------------------------------------------------------------------------
#if MotherBoard==3
#define KnowBoard 1
#define PiBot 1   //////******* for feture use
#define PI_HD_ESN "PI_PBHD0002_0"
#define CPU_MAP_ATMEGA2560 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'PiBot Mega2560' selected from the 'Tools -> Boards' menu.
#endif

#if PiBot_HD_REV==1
// Define step pulse output pins.    ////*****PiBot PIN is compatible with Ardunio
/////////////////###################################
#define X_STEP_PIN         4    ///******set for PiBot Board
#define Y_STEP_PIN         56
#define Z_STEP_PIN         59

// Define step direction output pins. 
///////////////////////////######################################
#define X_DIR_PIN          17
#define Y_DIR_PIN          55
#define Z_DIR_PIN          58

// Define stepper driver enable/disable output pins.
//////////////////////////############################*****************
#define X_ENABLE_PIN       16   /////***if only one enbale pin set  X,Y,Z with same pinNUM
#define Y_ENABLE_PIN       54
#define Z_ENABLE_PIN       57

// define  endstop input pins
//////////////////////////############################*****************
#ifdef limit_int_style
#define X_MIN_PIN          10    ///////****not work yet because of PCInt not here
#define Y_MIN_PIN          11
#define Z_MIN_PIN          13
#else
#define X_MIN_PIN          37    ///////****not work yet because of PCInt not here
#define Y_MIN_PIN          36
#define Z_MIN_PIN          35
#endif
//Feature the second stepper driver for the CNC machine
/////////////////////////////#########################*****************
#if FEATURE_SECOND_X
#define X2_STEP_PIN        24
#define X2_DIR_PIN	       23
#define X2_ENABLE_PIN      22
#endif

#if FEATURE_SECOND_Y
#define Y2_STEP_PIN        27
#define Y2_DIR_PIN	       26
#define Y2_ENABLE_PIN      25
#endif

#if FEATURE_SECOND_Z
#define Z2_STEP_PIN        15
#define Z2_DIR_PIN	       14
#define Z2_ENABLE_PIN      39
#endif

//////******interrupt related move into system.h   

// Define user-control pinouts (cycle start, reset, feed hold) input pins.
// NOTE: All pinouts pins must be on the PCINT function Port and change the related setting in System.h
//////////////////////////#############################*****************
#define CYCLE_INPUT_PIN      67  ////*****PCINT21 22 23; digtal: 67 68 69 
#define FEED_HOLD_INPUT_PIN  68  //  if change int port, change in System.h too    
#define RESET_INPUT_PIN      69  //  if change int port, change in System.h too 

// Define probe switch input pin.
//////////////////////////#############################*****************
#define PI_PROBE_PIN	      64	  /////###******** A10    ////*****PCINT18   same with 3D
//////////////////////////#############################*****************

/////###################*******************************

// Define spindle enable and spindle direction output pins.
//////////////////////////############################*****************
#define SPINDLE_ENABLE_PIN  3
#define SPINDLE_DIR_PIN     6
//////////////////////////#############################*****************

// Define flood and mist coolant enable output pins.
//////////////////////////#############################*****************
#define COOLANT_FLOOD_PIN   7    /////****fan
#ifdef ENABLE_M7     // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_PIN    2    /////***not  func yet, reserved for later use
#endif
//////////////////////////#############################*****************

// Start of PWM & Stepper Enabled Spindle
///////////////////////////#########################********************
#if VARIABLE_SPINDLE
//define PWM output pin
#define SPINDLE_PWM_PIN  9
#endif // End of VARIABLE_SPINDLE
///////////////////////////#########################********************
#endif  ////****HD 1

//////////////////////////////////////////////*************************************************

#if PiBot_HD_REV==2
// define output pins for system motion control
////////////////////####################******************************
#define X_STEP_PIN         24    ///******set for PiBot Board
#define X_DIR_PIN          23
#define X_ENABLE_PIN       22   /////***if only one enbale pin set  X,Y,Z with same pinNUM

#define Y_STEP_PIN         27
#define Y_DIR_PIN          26
#define Y_ENABLE_PIN       25

#define Z_STEP_PIN         15
#define Z_DIR_PIN          14
#define Z_ENABLE_PIN       39

// define  endstop input pins
//////////////////////////############################*****************
#ifdef limit_int_style     //////***needed  intterupte declare, and service function
#define X_MIN_PIN          62    ///////**** PCINT2     16  18  20
#define Y_MIN_PIN          64    ///////**** PCINT2     16  18  20
#define Z_MIN_PIN          66    ///////**** PCINT2    
////////***** not  use yet
#define X_MAX_PIN          63    ///////**** PCINT2     17  19  21
#define Y_MAX_PIN          65    ///////**** PCINT2     17  19  21
#define Z_MAX_PIN          67    ///////**** PCINT2    
#else     ///****PiBot Rev 2.0 all limit pins is int pin
#define X_MIN_PIN          62    ///////**** PCINT2     16  18  20
#define Y_MIN_PIN          64    ///////**** PCINT2     16  18  20
#define Z_MIN_PIN          66    ///////**** PCINT2    
////////***** not  use yet
#define X_MAX_PIN          63    ///////**** PCINT2     17  19  21
#define Y_MAX_PIN          65    ///////**** PCINT2     17  19  21
#define Z_MAX_PIN          67    ///////**** PCINT2    
#endif
//Feature the second stepper driver for the CNC machine
/////////////////////////////#########################*****************
#if FEATURE_SECOND_X         /////**** the fifth Driver use for Second Stepper Motor
#define X2_STEP_PIN        35    // if Use more than one Second feature, please make sure donot use the repeated Pins
#define X2_DIR_PIN	   34
#define X2_ENABLE_PIN      33
#endif

#if FEATURE_SECOND_Y      /////**** the fifth Driver use for Second Stepper Motor
#define Y2_STEP_PIN        35
#define Y2_DIR_PIN	   34
#define Y2_ENABLE_PIN      33
#endif

#if FEATURE_SECOND_Z         /////**** the fifth Driver use for Second Stepper Motor
#define Z2_STEP_PIN        35
#define Z2_DIR_PIN	   34
#define Z2_ENABLE_PIN      33
#endif

//////****** interrupt related move into system.h   

// Define user-control pinouts (cycle start, reset, feed hold) input pins.
// NOTE: All pinouts pins must be on the PCINT function Port and change the related setting in System.h
//////////////////////////#############################*****************
#define CYCLE_INPUT_PIN      12  ////*****PCINT0   5 6 7   
#define FEED_HOLD_INPUT_PIN  13  //  if change int port, change in System.h too    
#define RESET_INPUT_PIN      11  //  if change int port, change in System.h too 

// Define probe switch input pin.
//////////////////////////#############################*****************
#define PI_PROBE_PIN	     10	  /////###******** A10    ////*****PCINT0   4
//////////////////////////#############################*****************

/////###################*******************************

// Define spindle enable and spindle direction output pins.
//////////////////////////############################*****************
#define SPINDLE_ENABLE_PIN  48
#define SPINDLE_DIR_PIN     49
//////////////////////////#############################*****************

// Define flood and mist coolant enable output pins.
//////////////////////////#############################*****************
#define COOLANT_FLOOD_PIN   3    /////****fan  PWM3
#ifdef ENABLE_M7     // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_PIN    8    /////***not  func yet, reserved for later use  PWM8
#endif
//////////////////////////#############################*****************

// Start of PWM & Stepper Enabled Spindle
///////////////////////////#########################********************
#if VARIABLE_SPINDLE
//define PWM output pin
#define SPINDLE_PWM_PIN  9      //////****  PWM9
#endif // End of VARIABLE_SPINDLE
///////////////////////////#########################********************

#endif   /////***end HD 2


#endif ////##end pibot MB 3



//#############################################################
//*    (Arduino Uno)
//#############################################################
#if MotherBoard==1
#define KnowBoard 1
#define CPU_MAP_ATMEGA328P true 
#ifndef __AVR_ATmega328P__
#error Oops!  Make sure you have 'Arduino Uno' selected from the 'Tools -> Boards' menu.
#endif

  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD
  #define X_STEP_BIT      2  // Uno Digital Pin 2
  #define Y_STEP_BIT      3  // Uno Digital Pin 3
  #define Z_STEP_BIT      4  // Uno Digital Pin 4
  #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT   5  // Uno Digital Pin 5
  #define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
  #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
  #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (pinout).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
  #if VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
  #else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
  #endif
   #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR    DDRB
  #define SPINDLE_ENABLE_PORT   PORTB
  #if VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
  #else
    #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
  #endif  
  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
  #endif  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRC
  #define PINOUT_PIN       PINC
  #define PINOUT_PORT      PORTC
  #define PIN_RESET        0  // Uno Analog Pin 0
  #define PIN_FEED_HOLD    1  // Uno Analog Pin 1
  #define PIN_CYCLE_START  2  // Uno Analog Pin 2
  #define PINOUT_INT       PCIE1  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT1_vect
  #define PINOUT_PCMSK     PCMSK1 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))
  
  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5  // Uno Analog Pin 5
  #define PROBE_MASK      (1<<PROBE_BIT)

  
  #if VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    #define TCCRA_REGISTER	 TCCR2A
    #define TCCRB_REGISTER	 TCCR2B
    #define OCR_REGISTER     OCR2A

    #define COMB_BIT	     COM2A1
    #define WAVE0_REGISTER	 WGM20
    #define WAVE1_REGISTER	 WGM21
    #define WAVE2_REGISTER	 WGM22
    #define WAVE3_REGISTER	 WGM23

    // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
    #define SPINDLE_PWM_DDR	  SPINDLE_ENABLE_DDR
    #define SPINDLE_PWM_PORT  SPINDLE_ENABLE_PORT
    #define SPINDLE_PWM_BIT	  SPINDLE_ENABLE_BIT // Shared with SPINDLE_ENABLE.
  #endif // End of VARIABLE_SPINDLE

#endif



//#############################################################
//*  Arduino Mega 2560 R3  or Mega ADK
//#############################################################
#if MotherBoard==2
#define KnowBoard 1
#define CPU_MAP_ATMEGA2560 true
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega2560' selected from the 'Tools -> Boards' menu.
#endif

  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Increase Buffers to make use of extra SRAM
  //#define RX_BUFFER_SIZE		256
  //#define TX_BUFFER_SIZE		128
  //#define BLOCK_BUFFER_SIZE	36
  //#define LINE_BUFFER_SIZE	100

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR      DDRA
  #define STEP_PORT     PORTA
  #define STEP_PIN      PINA
  #define X_STEP_BIT     0  ///***22   2 // MEGA2560 Digital Pin 24
  #define Y_STEP_BIT     1  ///***23   3 // MEGA2560 Digital Pin 25
  #define Z_STEP_BIT     2  ///***24   4 // MEGA2560 Digital Pin 26
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR      DDRA
  #define DIRECTION_PORT     PORTA
  #define DIRECTION_PIN      PINA
  #define X_DIRECTION_BIT  3  ///***25    5 // MEGA2560 Digital Pin 27
  #define Y_DIRECTION_BIT  4  ///***26   6 // MEGA2560 Digital Pin 28
  #define Z_DIRECTION_BIT  5  ///***27   7 // MEGA2560 Digital Pin 29
  #define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR   DDRG    ///****B->G
  #define STEPPERS_DISABLE_PORT  PORTG
  ///////*****************************
  #define STEPPERS_DISABLE_PIN  PING
  //////////////////*********************
  #define STEPPERS_DISABLE_BIT   5 ///***4  7  // MEGA2560 Digital Pin 13
  #define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB         ///****B     C
  #define LIMIT_PORT      PORTB
  #define LIMIT_PIN       PINB
  #define X_LIMIT_BIT     4 // MEGA2560 Digital Pin 10   0 ////****37
  #define Y_LIMIT_BIT     5 // MEGA2560 Digital Pin 11   1 ////****36
  #define Z_LIMIT_BIT     7 //12->13  6 // MEGA2560 Digital Pin 12   2 ////****35 
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR   DDRH
  #define SPINDLE_ENABLE_PORT  PORTH
  #define SPINDLE_ENABLE_BIT   3 // MEGA2560 Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   5 /////****3  3 // MEGA2560 Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   4////*****7   5 // MEGA2560 Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRH
    #define COOLANT_MIST_PORT  PORTH
    #define COOLANT_MIST_BIT   6 // MEGA2560 Digital Pin 9
  #endif  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRK
  #define PINOUT_PIN       PINK
  #define PINOUT_PORT      PORTK
  #define PIN_RESET        0  // MEGA2560 Analog Pin 8   ////*****62
  #define PIN_FEED_HOLD    1  // MEGA2560 Analog Pin 9  ////*****63
  #define PIN_CYCLE_START  2  // MEGA2560 Analog Pin 10 ////*****64
  #define PINOUT_INT       PCIE2  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT2_vect
  #define PINOUT_PCMSK     PCMSK2 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))

  // Define probe switch input pin.
  #define PROBE_DDR       DDRK
  #define PROBE_PIN       PINK
  #define PROBE_PORT      PORTK
  #define PROBE_BIT       3  // MEGA2560 Analog Pin 11  ////*****65
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
  #if VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    // Set Timer up to use TIMER2B which is attached to Digital Pin 9
    #define TCCRA_REGISTER		TCCR2A
    #define TCCRB_REGISTER		TCCR2B
    #define OCR_REGISTER		OCR2B

    #define COMB_BIT			COM2B1
    #define WAVE0_REGISTER		WGM20
    #define WAVE1_REGISTER		WGM21
    #define WAVE2_REGISTER		WGM22
    #define WAVE3_REGISTER		WGM23

    #define SPINDLE_PWM_DDR		DDRH
    #define SPINDLE_PWM_PORT    PORTH
    #define SPINDLE_PWM_BIT		6 // MEGA2560 Digital Pin 9
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------
/* 
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and paste one of the default cpu map
  // settings above and modify it to your needs. Then, make sure the defined name is also
  // changed in the config.h file.
#endif
*/

#if KnowBoard!=1
#error Oops! Please select an available Controller Board from the 'Tools -> Boards' menu.
#endif


#endif
