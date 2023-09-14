/*
 * Common functions for all the trackers
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

#ifndef TRACKERLIB_H
#define TRACKERLIB_H



#include <stdint.h>

#define MCU_H 16

int init_serial(const char *path);
// packed RGB output
void decompress_jpeg(uint8_t *picture_data, 
    int picture_size,
    int *decoded_w,
    int *decoded_h,
    uint8_t **rows);
// planar YUV420 output
void decompress_jpeg_yuv(uint8_t *picture_data, 
    int picture_size,
    int *decoded_w,
    int *decoded_h,
    uint8_t *dst);

#endif

