/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#ifndef serial_h
#define serial_h


#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

#define SERIAL_NO_DATA 0xff

#ifdef ENABLE_XONXOFF
  #define RX_BUFFER_FULL 96 // XOFF high watermark
  #define RX_BUFFER_LOW 64 // XON low watermark
  #define SEND_XOFF 1
  #define SEND_XON 2
  #define XOFF_SENT 3
  #define XON_SENT 4
  #define XOFF_CHAR 0x13
  #define XON_CHAR 0x11
#endif

void serial_init();

// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data);

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read();

// Reset and empty data in read buffer. Used by e-stop and reset.
void serial_reset_read_buffer();

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count();

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count();

#endif
