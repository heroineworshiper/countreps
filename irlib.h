/*
 * tracking camera
 * Copyright (C) 2019-2023 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

#ifndef IRLIB_H
#define IRLIB_H

const uint8_t CODE[] = { 0xac, 0xf4, 0x56, 0x6a };

// from comcast.X
#define NO_BUTTON 0
#define STAR 1
#define I_BUTTON 2
#define UP 3
#define BACK 4
#define MIC 5
#define HOME 6
#define LEFT 7
#define RIGHT 8
#define SELECT_BUTTON 9
#define DOWN 10
#define TV_INPUT 11
#define POWER 12
#define PLUS 13
#define MUTE 14
#define MINUS 15
#define TOTAL_BUTTONS 16

// from servos.X
#define IR_LOW 0
#define IR_HIGH 1
#define IR_REPEAT 2
#define IR_TIMEOUT 3

// returns the button pressed, BUTTON_RELEASED, or -1
#define BUTTON_RELEASED 17
int process_code(uint8_t c);


#endif






