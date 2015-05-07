/*
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
   
   mainpage PiBot-Firmware for Arduino based GRBL
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2013 Sungeun K. Jeon
    Copyright (c) 2014 by Thomas
*/
/*
   Supported hardware from config.h
   	PiBot Controller Rev2.x   / 115200 bauds  (3 axis)
       Arduino.ino  / 115200 bauds  (3 axis)
	Arduino Mega 2560   / 115200 bauds  (3 axis)
   Supported sofware
        PiBot GRBL Controller V1.x
   	GRBL Controller V3.x

*/
/*

Implemented Codes

- Non-Modal Commands: G4, G10 L2, G10 L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
- Motion Modes: G0, G1, G2, G3, G38.1, G80
- Feed Rate Modes: G93, G94
- Unit Modes: G20, G21
- Distance Modes: G90, G91
- Plane Select Modes: G17, G18, G19
- Tool Length Offset Modes: G43.1, G49
- Coordinate System Modes: G54, G55, G56, G57, G58, G59
- Program Flow: M0, M1, M2, M30*
- Coolant Control: M7*, M8, M9
- Spindle Control: M3, M4, M5

*/

#include "PiBot.h"

void setup()
{
  main();
}

void loop()
{}
