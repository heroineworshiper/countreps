/*
 * Server for the tracking camera
 * Copyright (C) 2019-2021 Adam Williams <broadcast at earthling dot net>
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

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <pthread.h>
#include "tracker.h"
#include "jpeglib.h"
#include <setjmp.h>



#ifdef USE_SERVER

#define RECV_PORT 1234
#define SEND_PORT 1235
#define SOCKET_BUFSIZE 1024

// packet type
#define VIJEO 0x00
#define STATUS 0x01
#define KEYPOINTS 0x02


#define START_CODE0 0xff
#define START_CODE1 0xe7


uint8_t prev_error_flags = 0xff;

int recv_socket = -1;
int send_socket = -1;
uint32_t send_addr = 0;
sem_t data_ready;


// pointers to last sent buffers
// when the value is <= 0, the buffer is ready to accept new data
// when the value is > 0, it's being written
uint8_t *keypoint_buffer2 = 0;
int keypoint_size2 = 0;
int current_input2 = -1;
uint8_t status_buffer[HEADER_SIZE + 32];
int status_size = 0;

// destination for JPEG compression
#define MAX_JPEG 0x100000
uint8_t vijeo_buffer[MAX_JPEG];
int vijeo_size = 0;
pthread_mutex_t www_mutex;


struct my_jpeg_error_mgr {
  struct jpeg_error_mgr pub;	/* "public" fields */
  jmp_buf setjmp_buffer;	/* for return to caller */
};

static struct my_jpeg_error_mgr my_jpeg_error;

typedef struct 
{
	struct jpeg_destination_mgr pub; /* public fields */

	JOCTET *buffer;		/* Pointer to buffer */
} my_destination_mgr;


METHODDEF(void) init_destination(j_compress_ptr cinfo)
{
  	my_destination_mgr *dest = (my_destination_mgr*)cinfo->dest;

/* Set the pointer to the preallocated buffer */
    vijeo_size = 0;
  	dest->buffer = vijeo_buffer + HEADER_SIZE;
  	dest->pub.next_output_byte = dest->buffer;
  	dest->pub.free_in_buffer = MAX_JPEG - HEADER_SIZE;
}


/*
 * Terminate destination --- called by jpeg_finish_compress
 * after all data has been written.  Usually needs to flush buffer.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */
METHODDEF(void) term_destination(j_compress_ptr cinfo)
{
/* Just get the length */
	my_destination_mgr *dest = (my_destination_mgr*)cinfo->dest;
	vijeo_size = MAX_JPEG - HEADER_SIZE - dest->pub.free_in_buffer;
}

/*
 * Empty the output buffer --- called whenever buffer fills up.
 *
 * In typical applications, this should write the entire output buffer
 * (ignoring the current state of next_output_byte & free_in_buffer),
 * reset the pointer & count to the start of the buffer, and return TRUE
 * indicating that the buffer has been dumped.
 *
 * In applications that need to be able to suspend compression due to output
 * overrun, a FALSE return indicates that the buffer cannot be emptied now.
 * In this situation, the compressor will return to its caller (possibly with
 * an indication that it has not accepted all the supplied scanlines).  The
 * application should resume compression after it has made more room in the
 * output buffer.  Note that there are substantial restrictions on the use of
 * suspension --- see the documentation.
 *
 * When suspending, the compressor will back up to a convenient restart point
 * (typically the start of the current MCU). next_output_byte & free_in_buffer
 * indicate where the restart point will be if the current call returns FALSE.
 * Data beyond this point will be regenerated after resumption, so do not
 * write it out when emptying the buffer externally.
 */

METHODDEF(boolean) empty_vijeo_buffer(j_compress_ptr cinfo)
{
/* Allocate a bigger buffer. */
	my_destination_mgr *dest = (my_destination_mgr*)cinfo->dest;

//printf("empty_vijeo_buffer %d %d\n", __LINE__, dest->pub.free_in_buffer);
	dest->buffer = vijeo_buffer + HEADER_SIZE;
	dest->pub.next_output_byte = dest->buffer;
	dest->pub.free_in_buffer = MAX_JPEG - HEADER_SIZE;
	return TRUE;
}



void compress_jpeg()
{
	struct jpeg_compress_struct cinfo;
	cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
    jpeg_create_compress(&cinfo);
    cinfo.image_width = CAM_W;
    cinfo.image_height = CAM_H;
    cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 80, 0);
    cinfo.dct_method = JDCT_IFAST;
    cinfo.raw_data_in = TRUE;
    my_destination_mgr *dest;
    if(cinfo.dest == NULL) 
	{
/* first time for this JPEG object? */
      	cinfo.dest = (struct jpeg_destination_mgr *)
    		(*cinfo.mem->alloc_small)((j_common_ptr)&cinfo, 
				JPOOL_PERMANENT,
				sizeof(my_destination_mgr));
	}

	dest = (my_destination_mgr*)cinfo.dest;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_vijeo_buffer;
	dest->pub.term_destination = term_destination;

    int mcu_h = (int)(CAM_H / 16) * 16;
    if(mcu_h < CAM_H) mcu_h += 16;

    unsigned char **mcu_rows[3];
    mcu_rows[0] = new uint8_t*[mcu_h];
    mcu_rows[1] = new uint8_t*[mcu_h / 2];
    mcu_rows[2] = new uint8_t*[mcu_h / 2];

// pack Y with height scaling
    for(int i = 0; i < CAM_H; i++)
    {
        uint8_t *src = hdmi_rows[current_input2][i * HDMI_H / CAM_H];
        uint8_t *dst;
        mcu_rows[0][i] = dst = new uint8_t[CAM_W];
        for(int j = 0; j < CAM_W; j++)
        {
            *dst++ = src[0];
            src += 2;
        }
    }

// pad Y
    for(int i = CAM_H; i < mcu_h; i++)
    {
        uint8_t *dst;
        mcu_rows[0][i] = dst = new uint8_t[CAM_W];
        memset(dst, 0, CAM_W);
    }

// pack UV with height scaling
    for(int i = 0; i < CAM_H / 2; i++)
    {
        uint8_t *src = hdmi_rows[current_input2][i * 2 * HDMI_H / CAM_H];
        uint8_t *dst_u;
        uint8_t *dst_v;
        mcu_rows[1][i] = dst_u = new uint8_t[CAM_W / 2];
        mcu_rows[2][i] = dst_v = new uint8_t[CAM_W / 2];
        for(int j = 0; j < CAM_W / 2; j++)
        {
            *dst_u++ = src[1];
            *dst_v++ = src[3];
            src += 4;
        }
    }

// pad UV
    for(int i = CAM_H / 2; i < mcu_h / 2; i++)
    {
        uint8_t *dst_u;
        uint8_t *dst_v;
        mcu_rows[1][i] = dst_u = new uint8_t[CAM_W / 2];
        mcu_rows[2][i] = dst_v = new uint8_t[CAM_W / 2];
        memset(dst_u, 0, CAM_W / 2);
        memset(dst_v, 0, CAM_W / 2);
    }

    jpeg_start_compress(&cinfo, TRUE);
    while(cinfo.next_scanline < cinfo.image_height)
	{
        uint8_t **mcu_rows2[3];
        mcu_rows2[0] = new uint8_t*[16];
        mcu_rows2[1] = new uint8_t*[8];
        mcu_rows2[2] = new uint8_t*[8];
        for(int i = 0; i < 16; i++)
            mcu_rows2[0][i] = mcu_rows[0][cinfo.next_scanline + i];
        for(int i = 0; i < 8; i++)
        {
            mcu_rows2[1][i] = mcu_rows[1][cinfo.next_scanline / 2 + i];
            mcu_rows2[2][i] = mcu_rows[2][cinfo.next_scanline / 2 + i];
        }
        jpeg_write_raw_data(&cinfo,
            mcu_rows2,
            16);
        delete [] mcu_rows2[0];
        delete [] mcu_rows2[1];
        delete [] mcu_rows2[2];
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    for(int i = 0; i < mcu_h; i++)
    {
        delete [] mcu_rows[0][i];
    }
    for(int i = 0; i < mcu_h / 2; i++)
    {
        delete [] mcu_rows[1][i];
        delete [] mcu_rows[2][i];
    }
    delete [] mcu_rows[0];
    delete [] mcu_rows[1];
    delete [] mcu_rows[2];
}




int send_packet(int type, 
    uint8_t *data, // pointer to start of header
    int bytes) // size not including header
{
    int size = HEADER_SIZE + bytes;

    data[0] = START_CODE0;
    data[1] = START_CODE1;
    data[2] = type;
    data[3] = 0;
    data[4] = bytes & 0xff;
    data[5] = (bytes >> 8) & 0xff;
    data[6] = (bytes >> 16) & 0xff;
    data[7] = (bytes >> 24) & 0xff;

    int result = -1;
    pthread_mutex_lock(&www_mutex);
    result = write(send_socket, data, size);
    pthread_mutex_unlock(&www_mutex);
    return result;
}

void send_status()
{
    int offset = HEADER_SIZE;
    status_buffer[offset++] = current_operation;
    status_buffer[offset++] = 0;

    int tmp = (int)pan;
    status_buffer[offset++] = tmp & 0xff;
    status_buffer[offset++] = (tmp >> 8) & 0xff;
    status_buffer[offset++] = (tmp >> 16) & 0xff;
    status_buffer[offset++] = (tmp >> 24) & 0xff;

    tmp = (int)tilt;
    status_buffer[offset++] = tmp & 0xff;
    status_buffer[offset++] = (tmp >> 8) & 0xff;
    status_buffer[offset++] = (tmp >> 16) & 0xff;
    status_buffer[offset++] = (tmp >> 24) & 0xff;

    tmp = (int)start_pan;
    status_buffer[offset++] = tmp & 0xff;
    status_buffer[offset++] = (tmp >> 8) & 0xff;
    status_buffer[offset++] = (tmp >> 16) & 0xff;
    status_buffer[offset++] = (tmp >> 24) & 0xff;

    tmp = (int)start_tilt;
    status_buffer[offset++] = tmp & 0xff;
    status_buffer[offset++] = (tmp >> 8) & 0xff;
    status_buffer[offset++] = (tmp >> 16) & 0xff;
    status_buffer[offset++] = (tmp >> 24) & 0xff;

    status_buffer[offset++] = pan_sign;
    status_buffer[offset++] = tilt_sign;
    status_buffer[offset++] = lens;
    status_buffer[offset++] = landscape;
    status_buffer[offset++] = error_flags;

    prev_error_flags = error_flags;

    status_size = offset - HEADER_SIZE;
// wake up the writer
    sem_post(&data_ready);
            
//    printf("send_status %d error_flags=%d\n", __LINE__, error_flags);
//    joinBodyParts(connection, STATUS, buffer, 23);
}

void send_error()
{
    pthread_mutex_lock(&www_mutex);
//     printf("send_error %d error_flags=%d prev_error_flags=%d\n", 
//         __LINE__,
//         error_flags,
//         prev_error_flags);
    if(error_flags != prev_error_flags)
    {
		send_status();
    }
    pthread_mutex_unlock(&www_mutex);
}


void send_vijeo(int current_input)
{
    pthread_mutex_lock(&www_mutex);
    current_input2 = current_input;
    sem_post(&data_ready);
    pthread_mutex_unlock(&www_mutex);
}


void send_keypoints(uint8_t *buffer, int size)
{
    pthread_mutex_lock(&www_mutex);
    keypoint_buffer2 = buffer;
    keypoint_size2 = size;
    sem_post(&data_ready);
    pthread_mutex_unlock(&www_mutex);
}


void* web_server_reader(void *ptr)
{
	unsigned char buffer[SOCKET_BUFSIZE];
	while(1)
	{
//        int bytes_read = read(recv_socket, buffer, SOCKET_BUFSIZE);
        struct sockaddr_in peer_addr;
        socklen_t peer_addr_len = sizeof(struct sockaddr_in);
//printf("web_server_reader %d\n", __LINE__);
        int bytes_read = recvfrom(recv_socket,
            buffer, 
            SOCKET_BUFSIZE, 
            0,
            (struct sockaddr *) &peer_addr, 
            &peer_addr_len);
//printf("web_server_reader %d\n", __LINE__);
        if(send_addr != peer_addr.sin_addr.s_addr)
        {
            printf("web_server_reader %d: new connection\n", __LINE__);
            printf("web_server_reader %d peer_addr=%08x\n", 
                __LINE__, 
                peer_addr.sin_addr.s_addr);
            if(send_socket >= 0)
                close(send_socket);
            send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            send_addr = peer_addr.sin_addr.s_addr;
            peer_addr.sin_port = htons((unsigned short)SEND_PORT);
            connect(send_socket, 
		        (struct sockaddr*)&peer_addr, 
		        peer_addr_len);
        }

        int i;
        for(i = 0; i < bytes_read; i++)
        {
            int c = buffer[i];
            printf("web_server_reader %d '%c'\n", __LINE__, buffer[i]);

// do everything GUI::keypress_event does
            if(c == '*')
            {
                send_status();
            }
            else
            if(current_operation == STARTUP)
            {
                if(c == ' ')
                {
                    current_operation = CONFIGURING;

                    send_status();

                    do_startup();
                }
            }
            else
            if(current_operation == CONFIGURING)
            {
                int need_write_servos = 0;

                switch(c)
                {
                    case 'q':
                        stop_servos();
                        ::save_defaults();
                        current_operation = STARTUP;
                        break;

                    case '\n':
                        ::save_defaults();
                        current_operation = TRACKING;
                        break;

                    case 'w':
                        tilt += lenses[lens].tilt_step * tilt_sign;
                        start_pan = pan;
                        start_tilt = tilt;
                        need_write_servos = 1;
                        break;

                    case 's':
                        tilt -= lenses[lens].tilt_step * tilt_sign;
                        start_pan = pan;
                        start_tilt = tilt;
                        need_write_servos = 1;
                        break;

                    case 'a':
                        pan -= lenses[lens].pan_step * pan_sign;
                        start_pan = pan;
                        start_tilt = tilt;
                        need_write_servos = 1;
                        break;

                    case 'd':
                        pan += lenses[lens].pan_step * pan_sign;
                        start_pan = pan;
                        start_tilt = tilt;
                        need_write_servos = 1;
                        break;

                    case 'c':
                        pan = start_pan;
                        tilt = start_tilt;
                        need_write_servos = 1;
                        break;

                    case 't':
                        tilt_sign *= -1;
                        ::save_defaults();
                        break;

                    case 'p':
                        pan_sign *= -1;
                        ::save_defaults();
                        break;

//                         case 'r':
//                             landscape = !landscape;
//                             ::save_defaults();
//                             break;

                    case 'r':
                        if(!landscape)
                        {
                            landscape = 1;
                            ::save_defaults();
                        }
                        break;

                    case 'R':
                        if(landscape)
                        {
                            landscape = 0;
                            ::save_defaults();
                        }
                        break;

                    case 'l':
                        lens++;
                        if(lens >= TOTAL_LENSES)
                        {
                            lens = 0;
                        }
                        ::save_defaults();
                        break;
                }

                if(need_write_servos)
                {
                    write_servos(1);
                }
                send_status();
            }
            else
            if(current_operation == TRACKING)
            {
                switch(c)
                {
                    case 'Q':
                        stop_servos();
                        current_operation = STARTUP;
                        send_status();
                        break;

                    case 'b':
                        current_operation = CONFIGURING;
                        send_status();
                        break;
                }
            }
        }
    }
}


void* web_server_writer(void *ptr)
{
	unsigned char buffer[SOCKET_BUFSIZE];
	int i;

	while(1)
	{
// wait for data to write
        sem_wait(&data_ready);


// no client
        if(send_socket < 0)
            continue;

        if(status_size)
        {
            send_packet(STATUS, status_buffer, status_size);
            status_size = 0;
        }

        if(keypoint_size2)
        {
            send_packet(KEYPOINTS, keypoint_buffer2, keypoint_size2);
            keypoint_size2 = 0;
        }

        if(current_input2 >= 0)
        {
// compress it
            compress_jpeg();
            send_packet(VIJEO, vijeo_buffer, vijeo_size);
            current_input2 = -1;
        }
	}
}




void init_server()
{
	recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	int reuseon = 1;
    setsockopt(recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuseon, sizeof(reuseon));
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(RECV_PORT);

    int result = bind(recv_socket, (struct sockaddr *) &addr, sizeof(addr));
	if(result)
	{
		printf("init_server %d: bind port %d failed\n", __LINE__, RECV_PORT);
	}



	sem_init(&data_ready, 0, 0);

	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_t tid;

	pthread_create(&tid, 
		&attr, 
		web_server_writer, 
		0);
	pthread_create(&tid, 
		&attr, 
		web_server_reader, 
		0);
}





#endif // USE_SERVER
















