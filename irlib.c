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

// parse IR codes from the servo/IR board



#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "irlib.h"

#define CODE_BITS 32
#define CODE_BYTES 4
static int offset = 0;
static uint8_t buffer[CODE_BYTES];
static int button = NO_BUTTON;

const uint8_t *button_strings[] = 
{
    "NO_BUTTON",
    "STAR",
    "I_BUTTON",
    "UP",
    "BACK",
    "MIC",
    "HOME",
    "LEFT",
    "RIGHT",
    "SELECT",
    "DOWN",
    "TV_INPUT",
    "POWER",
    "PLUS",
    "MUTE",
    "MINUS",
};

void shift_buffer()
{
    buffer[0] <<= 1;
    if((buffer[1] & 0x80)) buffer[0] |= 1;
    buffer[1] <<= 1;
    if((buffer[2] & 0x80)) buffer[1] |= 1;
    buffer[2] <<= 1;
    if((buffer[3] & 0x80)) buffer[2] |= 1;
    buffer[3] <<= 1;
    offset++;
}

void process_code(uint8_t c)
{
//    printf("process_code %x\n", c);
    if(c == IR_TIMEOUT)
    {
        printf(" timeout\n");
        offset = 0;
    }
    else
    if(c == IR_REPEAT)
    {
        printf(".");
        fflush(stdout);
        offset = 0;
    }
    else
    if(c == IR_LOW)
    {
        shift_buffer();
    }
    else
    if(c == IR_HIGH)
    {
        shift_buffer();
        buffer[3] |= 1;
    }

    if(offset >= CODE_BITS)
    {
        int i;
        offset = 0;
        for(button = 0; button < TOTAL_BUTTONS; button++)
        {
            for(i = 0; i < CODE_BYTES; i++)
            {
                if((CODE[i] ^ button) != buffer[i]) break;
            }
            if(i >= CODE_BYTES) break;
        }
        if(button < TOTAL_BUTTONS)
        {
            printf("Got %s", button_strings[button]);
            fflush(stdout);
        }
        else
        {
            button = NO_BUTTON;
        }
    }
}




int init_serial(const char *path)
{
	struct termios term;
    printf("init_serial %d: opening %s\n", __LINE__, path);

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
	if(tcsetattr(fd, TCSANOW, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}

void main()
{
    int fd = init_serial("/dev/ttyACM0");
    if(fd < 0) fd = init_serial("/dev/ttyACM1");
	if(fd < 0) fd = init_serial("/dev/ttyACM2");
    if(fd >= 0)
    {
        uint8_t buffer;
        while(1)
        {
            int bytes_read = read(fd, &buffer, 1);
            if(bytes_read <= 0)
            {
                printf("main %d: board unplugged\n", __LINE__);
                close(fd);
                fd = -1;
                break;
            }
            else
            {
                process_code(buffer);
            }
        }
    }
}














