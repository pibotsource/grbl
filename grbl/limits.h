/*
  limits.h - code pertaining to limit-switches and performing the homing cycle
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

#ifndef limits_h
#define limits_h 


// Initialize the limits module
void limits_init();

void limits_disable();

// Perform one portion of the homing cycle based on the input settings.
void limits_go_home(uint8_t cycle_mask);

// Check for soft limit violations
void limits_soft_check(float *target);

#if MotherBoard==3
//////////************************** pibot endstop check status get
static inline bool isXMinEndstopHit();

static inline bool isYMinEndstopHit();

static inline bool isZMinEndstopHit();
///////////********************* new function for pibot  MAX encstop check
static inline bool isXMaxEndstopHit();

static inline bool isYMaxEndstopHit();

static inline bool isZMaxEndstopHit();
/////////####################################********thomas
#endif

#endif