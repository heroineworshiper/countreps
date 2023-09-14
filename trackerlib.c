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

#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "jpeglib.h"
#include "trackerlib.h"

int init_serial(const char *path)
{
	struct termios term;
    int verbose = 0;

	if(verbose) printf("init_serial %d: opening %s\n", __LINE__, path);

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		if(verbose) printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		if(verbose) printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}


/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
//	term.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
//	term.c_cflag |= CS8;
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	if(tcsetattr(fd, TCSANOW, &term))
	{
		if(verbose) printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}







typedef struct 
{
	struct jpeg_source_mgr pub;	/* public fields */

	JOCTET * buffer;		/* start of buffer */
	int bytes;             /* total size of buffer */
} jpeg_source_mgr_t;
typedef jpeg_source_mgr_t* jpeg_src_ptr;


struct my_jpeg_error_mgr {
  struct jpeg_error_mgr pub;	/* "public" fields */
  jmp_buf setjmp_buffer;	/* for return to caller */
};

static struct my_jpeg_error_mgr my_jpeg_error;

METHODDEF(void) init_source(j_decompress_ptr cinfo)
{
    jpeg_src_ptr src = (jpeg_src_ptr) cinfo->src;
}

METHODDEF(boolean) fill_input_buffer(j_decompress_ptr cinfo)
{
	jpeg_src_ptr src = (jpeg_src_ptr) cinfo->src;
#define   M_EOI     0xd9

	src->buffer[0] = (JOCTET)0xFF;
	src->buffer[1] = (JOCTET)M_EOI;
	src->pub.next_input_byte = src->buffer;
	src->pub.bytes_in_buffer = 2;

	return TRUE;
}


METHODDEF(void) skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
	jpeg_src_ptr src = (jpeg_src_ptr)cinfo->src;

	src->pub.next_input_byte += (size_t)num_bytes;
	src->pub.bytes_in_buffer -= (size_t)num_bytes;
}


METHODDEF(void) term_source(j_decompress_ptr cinfo)
{
}

METHODDEF(void) my_jpeg_output (j_common_ptr cinfo)
{
}


METHODDEF(void) my_jpeg_error_exit (j_common_ptr cinfo)
{
/* cinfo->err really points to a mjpeg_error_mgr struct, so coerce pointer */
  	struct my_jpeg_error_mgr* mjpegerr = (struct my_jpeg_error_mgr*) cinfo->err;

printf("my_jpeg_error_exit %d\n", __LINE__);
/* Always display the message. */
/* We could postpone this until after returning, if we chose. */
  	(*cinfo->err->output_message) (cinfo);

/* Return control to the setjmp point */
  	longjmp(mjpegerr->setjmp_buffer, 1);
}

static int check_sig(uint8_t *ptr)
{
    return !(
// HDMI
        (ptr[0] == 0xff && 
            ptr[1] == 0xd8 && 
            ptr[2] == 0xff && 
            ptr[3] == 0xdb) ||
// generalplus
        (ptr[0] == 0xff && 
            ptr[1] == 0xd8 && 
            ptr[2] == 0xff && 
            ptr[3] == 0xc0) ||
// gimp
        (ptr[0] == 0xff && 
            ptr[1] == 0xd8 && 
            ptr[2] == 0xff && 
            ptr[3] == 0xe0)
        );
}



void decompress_jpeg_yuv(uint8_t *picture_data, 
    int picture_size,
    int *decoded_w,
    int *decoded_h,
    uint8_t *dst)
{
    if(check_sig(picture_data))
        return;


	uint8_t **mcu_rows[3];
    mcu_rows[0] = (uint8_t**)malloc(MCU_H * sizeof(unsigned char*));
    mcu_rows[1] = (uint8_t**)malloc(MCU_H / 2 * sizeof(unsigned char*));
    mcu_rows[2] = (uint8_t**)malloc(MCU_H / 2 * sizeof(unsigned char*));

	struct jpeg_decompress_struct cinfo;
	cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
    my_jpeg_error.pub.error_exit = my_jpeg_error_exit;

    my_jpeg_error.pub.output_message = my_jpeg_output;
	jpeg_create_decompress(&cinfo);
	if(setjmp(my_jpeg_error.setjmp_buffer))
	{
        jpeg_destroy_decompress(&cinfo);
	    free(mcu_rows[0]);
	    free(mcu_rows[1]);
	    free(mcu_rows[2]);
        return;
    }
    cinfo.dct_method = JDCT_IFAST;

	cinfo.src = (struct jpeg_source_mgr*)
    	(*cinfo.mem->alloc_small)((j_common_ptr)&cinfo, 
        JPOOL_PERMANENT,
		sizeof(jpeg_source_mgr_t));
	jpeg_src_ptr src = (jpeg_src_ptr)cinfo.src;
	src->pub.init_source = init_source;
	src->pub.fill_input_buffer = fill_input_buffer;
	src->pub.skip_input_data = skip_input_data;
	src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
	src->pub.term_source = term_source;
	src->pub.bytes_in_buffer = picture_size;
	src->pub.next_input_byte = picture_data;
	src->buffer = picture_data;
	src->bytes = picture_size;

//printf("decompress_jpeg %d\n", __LINE__);
	jpeg_read_header(&cinfo, 1);
    cinfo.raw_data_out = TRUE;
	jpeg_start_decompress(&cinfo);

    *decoded_w = cinfo.output_width;
    *decoded_h = cinfo.output_height;

//printf("decompress_jpeg %d w=%d h=%d\n", __LINE__, *decoded_w, *decoded_h);

	while(cinfo.output_scanline < *decoded_h)
	{
		for(int i = 0; i < MCU_H; i++)
        {
            mcu_rows[0][i] = dst + *decoded_w * (cinfo.output_scanline + i);
        }
		for(int i = 0; i < MCU_H / 2; i++)
        {
            mcu_rows[1][i] = dst + 
                *decoded_w * *decoded_h + 
                (*decoded_w / 2) * ((cinfo.output_scanline + i * 2) / 2);
            mcu_rows[2][i] = dst + 
                *decoded_w * *decoded_h + 
                *decoded_w * *decoded_h / 4 + 
                (*decoded_w / 2) * ((cinfo.output_scanline + i * 2) / 2);
        }

//printf("decompress_jpeg %d %d\n", __LINE__, cinfo.output_scanline);
        jpeg_read_raw_data(&cinfo, 
			mcu_rows, 
			*decoded_h);
//printf("decompress_jpeg %d\n", __LINE__);
	}

    jpeg_destroy_decompress(&cinfo);
	free(mcu_rows[0]);
	free(mcu_rows[1]);
	free(mcu_rows[2]);
}


void decompress_jpeg(uint8_t *picture_data, 
    int picture_size,
    int *decoded_w,
    int *decoded_h,
    uint8_t **rows)
{
    *decoded_w = 0;
    *decoded_h = 0;

    if(check_sig(picture_data))
        return;


	struct jpeg_decompress_struct cinfo;
	cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
    my_jpeg_error.pub.error_exit = my_jpeg_error_exit;

    my_jpeg_error.pub.output_message = my_jpeg_output;
	jpeg_create_decompress(&cinfo);
	if(setjmp(my_jpeg_error.setjmp_buffer))
	{
        jpeg_destroy_decompress(&cinfo);
        return;
    }
	cinfo.src = (struct jpeg_source_mgr*)
    	(*cinfo.mem->alloc_small)((j_common_ptr)&cinfo, 
        JPOOL_PERMANENT,
		sizeof(jpeg_source_mgr_t));
	jpeg_src_ptr src = (jpeg_src_ptr)cinfo.src;
	src->pub.init_source = init_source;
	src->pub.fill_input_buffer = fill_input_buffer;
	src->pub.skip_input_data = skip_input_data;
	src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
	src->pub.term_source = term_source;
	src->pub.bytes_in_buffer = picture_size;
	src->pub.next_input_byte = picture_data;
	src->buffer = picture_data;
	src->bytes = picture_size;

	jpeg_read_header(&cinfo, 1);
	jpeg_start_decompress(&cinfo);

    *decoded_w = cinfo.output_width;
    *decoded_h = cinfo.output_height;
//printf("decompress_jpeg %d w=%d h=%d\n", __LINE__, *decoded_w, *decoded_h);

	while(cinfo.output_scanline < *decoded_h)
	{
		int num_scanlines = jpeg_read_scanlines(&cinfo, 
			&rows[cinfo.output_scanline],
			*decoded_h - cinfo.output_scanline);
	}

    jpeg_destroy_decompress(&cinfo);
}



