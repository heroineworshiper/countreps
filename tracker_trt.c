/*
 * 2 axis tracker using body_25 for FP16
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

// To build it:
// 
// make -f Makefile.trt
// 
// To run it:
// 
// ./tracker_trt


// OpenPose dependencies
#include "array.hpp"
#include "bodyPartConnectorBase.hpp"
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include "irlib.h"
#include "jpeglib.h"
#include <math.h>
#include <memory>
#include "nmsBase.hpp"
#include "point.hpp"
#include "poseParameters.hpp"
#include <pthread.h>
#include "resizeAndMergeBase.hpp"
#include "resize.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include "tracker.h"
#include "trackerlib.h"
#include "body25.h"
#include <unistd.h>

#define ENGINE_PORTRAIT "body25_240x160.engine"
#define ENGINE_LANDSCAPE "body25_144x256.engine"
//#define ENGINE "body25_128x224.engine"

// maximum animals to track in 2 axis mode
#define MAX_ANIMALS 2

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <libusb.h>
// video devices
#define DEVICE0 0
#define DEVICE1 4

// detect USB enumeration
// webcam has multiple IDs
#define WEBCAM_VID1 0x1b3f
#define WEBCAM_PID1 0x2202
#define WEBCAM_VID2 0x1b3f
#define WEBCAM_PID2 0x8301
#define HDMI_VID 0x534d
#define HDMI_PID 0x2109

#define VIDEO_BUFFERS 1


// analog values from arm_cam.c
// an offset is added to the ADC to get this range
#define ADC_CENTER 128
#define ADC_DEADBAND 5
#define ADC_MAX 64
#define MAX_ADC_OFFSET 32
// minimums
#define MIN_LEFT (ADC_CENTER + ADC_DEADBAND)
#define MIN_RIGHT (ADC_CENTER - ADC_DEADBAND)
// maximums
#define MAX_LEFT (ADC_CENTER + ADC_MAX)
#define MAX_RIGHT (ADC_CENTER - ADC_MAX)



int current_input = 0;
unsigned char *mmap_buffer[VIDEO_BUFFERS];

// copy of the mmap image sent to the server
// 1 is owned by the server for JPEG compression
// buffer for JPEG decompression
uint8_t *hdmi_image[INPUT_IMAGES];
uint8_t **hdmi_rows[INPUT_IMAGES];

// HDMI or TRUCK dimensions
int input_w;
int input_h;

// storage for packet header, keypoints & compressed frame
uint8_t vijeo_buffer[HEADER_SIZE + 2 + 2 + MAX_ANIMALS * BODY_PARTS * 4 + MAX_JPEG];

Engine landscape_engine;
Engine portrait_engine;


// write a buffer as a PPM image
void write_test(const char *path, 
    const void *data, 
    int framesx, 
    int framesy, 
    int w, 
    int h)
{
    float *test_buffer = (float*)malloc(framesx * 
        framesy * 
        w * 
        h * 
        sizeof(float));
    cudaMemcpy(test_buffer, 
        data, 
        framesx * 
            framesy * 
            w * 
            h * 
            sizeof(float), 
        cudaMemcpyDeviceToHost);

    FILE *test_fd = fopen(path, "w");
    fprintf(test_fd, "P6\n%d %d\n255\n", framesx * w, framesy * h);
    float max, min;
    int size = framesx * w * framesy * h;
    for(int i = 0; i < size; i++) 
    {
        if(i == 0) max = min = test_buffer[i];
        else
        if(test_buffer[i] > max) max = test_buffer[i];
        else
        if(test_buffer[i] < min) min = test_buffer[i];
    }

    uint8_t *buffer = (uint8_t*)calloc(1, size * 3);
    for(int n = 0; n < framesx * framesy; n++) 
    {
        int in_offset = n * w * h;
        int out_y = (n / framesx) * h;
        int out_x = (n % framesx) * w;
        int out_offset = (out_y * framesx * w + out_x) * 3;
        for(int i = 0; i < h; i++)
        {
            const float *in_row = test_buffer + in_offset + i * w;
            uint8_t *out_row = buffer + out_offset + i * framesx * w * 3;
            for(int j = 0; j < w; j++)
            {
                int v = (int)((*in_row++ - min) * 255 / (max - min));
                *out_row++ = v;
                *out_row++ = v;
                *out_row++ = v;
            }
        }
    }
    fwrite(buffer, 1, size * 3, test_fd);
    free(buffer);
    fclose(test_fd);
    printf("Wrote %s min=%f max=%f\n", path, min, max);
    free(test_buffer);
}


lens_t lenses[] = 
{
#ifdef USE_ATMEGA
    { 100, 100, 25,   25, 50, 50, 35, 4, 5 }, // 15mm
    { 100, 100, 12,   12, 50, 50, 25, 4, 5 }, // 28mm
    { 50,  50,  6,     6, 50, 50, 17, 4, 5 }, // 50mm
#endif

#ifdef USE_PIC
    { 150,  150, 18,   18, 38, 38, 35, 1, 5 }, // 15mm
    { 75,    75,  9,    9, 38, 38, 25, 4, 5 }, // 28mm
    { 38,    38,  5,    5, 38, 38, 17, 4, 5 }, // 50mm
#endif

};

// where to put the head based on head size (percentages of height)
// #define TOP_Y1 13
// #define HEAD_SIZE1 26
// 
// #define TOP_Y2 0
// #define HEAD_SIZE2 43

// scale from pixels to percent of screen size
#define TO_PERCENT_Y(y) ((y) * 100 / h)
#define TO_PERCENT_X(x) ((x) * 100 / w)
// scale from percent of screen size to pixels
#define FROM_PERCENT_Y(y) ((y) * h / 100)
#define FROM_PERCENT_X(x) ((x) * w / 100)


// the body parts as defined in 
// src/openpose/pose/poseParameters.cpp: POSE_BODY_25_BODY_PARTS
#define MODEL_NECK 1
#define MODEL_REYE 15
#define MODEL_LEYE 16
#define MODEL_REAR 17
#define MODEL_LEAR 18
#define MODEL_NOSE 0


#define MODEL_MIDHIP 8
#define MODEL_RHIP 9
#define MODEL_LHIP 12

#define MODEL_LKNEE 13
#define MODEL_RKNEE 10
#define MODEL_LELBOW 6
#define MODEL_RELBOW 3
#define MODEL_LSHOULDER 5
#define MODEL_RSHOULDER 2


#define MODEL_LANKLE 14
#define MODEL_RANKLE 11
#define MODEL_RHEEL 24
#define MODEL_LHEEL 21
#define MODEL_BIGTOE 22
#define MODEL_SMALLTOE 23

#define MODEL_WRIST2 4
#define BODY_PARTS 25
#define TOTAL_ZONES 4
#define HEAD_ZONE 0
#define NECK_ZONE 1
#define HIP_ZONE 2
#define FOOT_ZONE 3

static int zone0[] = {
    MODEL_REYE,
    MODEL_LEYE,
    MODEL_REAR,
    MODEL_LEAR,
    MODEL_NOSE,
    -1
};
static int zone1[] = {
    MODEL_NECK,
    MODEL_LSHOULDER,
    MODEL_RSHOULDER,
    -1
};
static int zone2[] = {
    MODEL_MIDHIP,
    MODEL_RHIP,
    MODEL_LHIP,
    -1
};
static int zone3[] = {
    MODEL_LANKLE,
    MODEL_RANKLE,
    MODEL_RHEEL,
    MODEL_LHEEL,
    MODEL_BIGTOE,
    MODEL_SMALLTOE,
    -1
};
        
// body parts comprising the zones
static int *zone_parts[] = { zone0, zone1, zone2, zone3 };

typedef struct
{
    int min_y;
    int max_y;
// total body parts detected
    int total;
// the body parts comprising this zone
    int *parts;
} zone_t;

typedef struct
{
    int orig_animal;
    int x1, y1, x2, y2;
// last detected size of the head for tilt tracking
    int head_size;
    zone_t zones[TOTAL_ZONES];
} body_t;

// value from the protobuf file
#define MAX_ANIMALS2 255
// entries in mPoseKeypoints sorted by size
static body_t bodies[MAX_ANIMALS2];

// raw PWM values
float pan = PAN0;
float tilt = TILT0;
float start_pan = pan;
float start_tilt = tilt;
int pan_sign = 1;
int tilt_sign = 1;
int lens = LENS_15;
int landscape = 1;
// truck cam with body25
int is_truck = 0;
// current ADC for truck cam
int adc = 0;
// feedback for truck cam
int deadband = 5;
int speed = 100;

static int servo_fd = -1;
static int frames = 0;
static FILE *ffmpeg_fd = 0;

int current_operation = STARTUP;
uint8_t error_flags = 0x00;


void* servo_reader(void *ptr)
{
//    printf("servo_reader %d\n", __LINE__);
    int servos = 0;

    while(1)
    {
// open the device
        if(servo_fd < 0)
        {
#ifdef __clang__
            servo_fd = init_serial("/dev/cu.usbserial-AL03OO1F");
#else
    #ifdef USE_ATMEGA
	        servo_fd = init_serial("/dev/ttyUSB0");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyUSB1");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyUSB2");
    #endif

    #ifdef USE_PIC
	        servo_fd = init_serial("/dev/ttyACM0");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM1");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM2");
    #endif

#endif
        }

        if(servo_fd >= 0)
        {
            if((error_flags & SERVO_ERROR))
            {
                printf("servo_reader %d: servos found\n", __LINE__);
            }
            error_flags &= ~SERVO_ERROR;
            send_error();

            uint8_t buffer;
            int repeat_count = 0;
            int prev_button = 0;
            while(1)
            {
                int bytes_read = read(servo_fd, &buffer, 1);
                if(bytes_read <= 0)
                {
                    printf("servo_reader %d: servos unplugged\n", __LINE__);
                    close(servo_fd);
                    servo_fd = -1;
                    break;
                }

#ifdef USE_IR
// process in IR library
                int result = process_code(buffer);
                int need_print_values = 0;
                int need_write_servos = 0;
                
                if(result == BUTTON_REPEAT &&
                    (prev_button == LEFT ||
                    prev_button == RIGHT ||
                    prev_button == UP ||
                    prev_button == DOWN))
                {
                    repeat_count++;
                    if(repeat_count >= 20)
                    {
                        if(((repeat_count - 20) % 5) == 0)
                        {
                            result = prev_button;
                        }
                    }
                }
                else
                if(result == BUTTON_RELEASED)
                {
                    repeat_count = 0;
                    prev_button = -1;
                }
                else
                if(result != -1)
                {
                    prev_button = result;
                }
                
                switch(result)
                {
                    case SELECT_BUTTON:
                        if(current_operation == STARTUP)
                        {
                            current_operation = CONFIGURING;
                            draw_config();
                            do_startup();
                        }
                        else
                        if(current_operation == CONFIGURING)
                        {
                            start_pan = pan;
                            start_tilt = tilt;
                            ::save_defaults();
                            current_operation = TRACKING;
                            do_tracking(1);
                        }
                        break;

                    case BACK:
                        if(current_operation == CONFIGURING)
                        {
                            current_operation = STARTUP;
                            draw_startup();
                            stop_servos();
                        }
                        else
                        if(current_operation == TRACKING)
                        {
                            current_operation = CONFIGURING;
                            draw_config();
                        }
                        break;

                    case HOME:
                        if(current_operation == CONFIGURING)
                        {
                            current_operation = STARTUP;
                            draw_startup();
                            stop_servos();
                        }
                        else
                        if(current_operation == TRACKING)
                        {
                            current_operation = STARTUP;
                            draw_startup();
                            stop_servos();
                        }
                        break;
                    
                    case LEFT:
                        pan -= lenses[lens].pan_step * pan_sign;
                        need_write_servos = 1;
                        need_print_values = 1;
                        break;
                    
                    case RIGHT:
                        pan += lenses[lens].pan_step * pan_sign;
                        need_write_servos = 1;
                        need_print_values = 1;
                        break;
                    
                    case UP:
                        tilt += lenses[lens].tilt_step * tilt_sign;
                        need_write_servos = 1;
                        need_print_values = 1;
                        break;
                    
                    case DOWN:
                        tilt -= lenses[lens].tilt_step * tilt_sign;
                        need_write_servos = 1;
                        need_print_values = 1;
                        break;
                    
                    case PLUS:
                        if(current_operation == CONFIGURING)
                        {
                            pan_sign *= -1;
                            need_print_values = 1;
                        }
                        break;
                    
                    case MINUS:
                        if(current_operation == CONFIGURING)
                        {
                            tilt_sign *= -1;
                            need_print_values = 1;
                        }
                        break;
                    
                    case STAR:
                        if(current_operation == CONFIGURING)
                        {
                            landscape = !landscape;
                            need_print_values = 1;
                        }
                        break;
                    
                    case I_BUTTON:
                        if(current_operation == CONFIGURING)
                        {
                            lens++;
                            if(lens >= TOTAL_LENSES)
                            {
                                lens = 0;
                            }
                            need_print_values = 1;
                        }
                        break;
                }
                
                if(need_print_values && current_operation == CONFIGURING)
                {
                    gui->lock_window();
                    gui->print_values(0);
                    gui->flash(1);
                    gui->unlock_window();
                }
                
                if(need_write_servos && current_operation != STARTUP)
                {
                    write_servos(1);
                }
#endif // USE_IR


            }
            //printf("%c", buffer);
            //fflush(stdout);
        }
        else
        {
            if(!(error_flags & SERVO_ERROR))
            {
                printf("servo_reader %d: servos not found\n", __LINE__);
            }
            error_flags |= SERVO_ERROR;
            send_error();
            sleep(1);
        }
    }
}

int init_servos()
{
    pthread_t x;
	pthread_create(&x, 
		0, 
		servo_reader, 
		0);


}

void write_servos(int use_pwm_limits)
{
	if(servo_fd >= 0)
	{
        if(is_truck)
        {
// 1 axis
            char buffer[1];
            buffer[0] = adc;
		    int temp = write(servo_fd, buffer, 1);
//printf("write_servos %d %d\n", __LINE__, adc);
        }
        else
        {
// 2 axis
    #define SYNC_CODE0 0xff
    #define SYNC_CODE1 0x2d
    #define SYNC_CODE2 0xd4
    #define SYNC_CODE3 0xe5
    #define BUFFER_SIZE 8

    // limits are absolute PWM limits
            if(use_pwm_limits)
            {
                CLAMP(pan, MIN_PWM, MAX_PWM);
                CLAMP(tilt, MIN_PWM, MAX_PWM);
            }
            else
    // limits are relative to the starting position
            {
                CLAMP(pan, start_pan - PAN_MAG, start_pan + PAN_MAG);
                CLAMP(tilt, start_tilt - TILT_MAG, start_tilt + TILT_MAG);
            }

            uint16_t pan_i = (uint16_t)pan;
            uint16_t tilt_i = (uint16_t)tilt;
		    char buffer[BUFFER_SIZE];
            buffer[0] = SYNC_CODE0;
            buffer[1] = SYNC_CODE1;
            buffer[2] = SYNC_CODE2;
            buffer[3] = SYNC_CODE3;
            buffer[4] = pan_i;
            buffer[5] = pan_i >> 8;
            buffer[6] = tilt_i;
            buffer[7] = tilt_i >> 8;

    //printf("write_servos %d %d %d\n", __LINE__, pan_i,  tilt_i);
		    int temp = write(servo_fd, buffer, BUFFER_SIZE);
        }
	}
}

void stop_servos()
{
	if(servo_fd >= 0 && !is_truck)
	{
#define SYNC_CODE0 0xff
#define SYNC_CODE1 0x2d
#define SYNC_CODE2 0xd4
#define SYNC_CODE3 0xe5
#define BUFFER_SIZE 8
		char buffer[BUFFER_SIZE];
        buffer[0] = SYNC_CODE0;
        buffer[1] = SYNC_CODE1;
        buffer[2] = SYNC_CODE2;
        buffer[3] = SYNC_CODE3;
        buffer[4] = 0;
        buffer[5] = 0;
        buffer[6] = 0;
        buffer[7] = 0;

// write it a few times to defeat UART initialization glitches
		int temp = write(servo_fd, buffer, BUFFER_SIZE);
        temp = write(servo_fd, buffer, BUFFER_SIZE);
        temp = write(servo_fd, buffer, BUFFER_SIZE);
        temp = write(servo_fd, buffer, BUFFER_SIZE);
    }    
}


void do_startup()
{
    if(!is_truck)
    {
// write it a few times to defeat UART initialization glitches
        write_servos(1);
        usleep(100000);
        write_servos(1);
        usleep(100000);
        write_servos(1);
        usleep(100000);
        write_servos(1);
    }
}


void load_defaults()
{
// HOME not available in /etc/rc.local
//    char *home = getenv("HOME");
    char *home = "/root";
    char string[TEXTLEN];
    sprintf(string, "%s/.tracker.rc", home);
    FILE *fd = fopen(string, "r");
    
    if(!fd)
    {
        printf("load_defaults %d: Couldn't open %s for reading\n", 
            __LINE__, 
            string);
        return;
    }
    
    while(!feof(fd))
    {
        if(!fgets(string, TEXTLEN, fd)) break;
// get 1st non whitespace character
        char *key = string;
        while((*key == ' ' ||
            *key == '\t' ||
            *key == '\n') && 
            *key != 0)
        {
            key++;
        }

// comment or empty
        if(*key == '#' || *key == 0)
        {
            continue;
        }
        
// get start of value
        char *value = key;
        while(*value != ' ' && 
            *value != '\t' && 
            *value != '\n' && 
            *value != 0)
        {
            value++;
        }


        while((*value == ' ' ||
            *value == '\t' ||
            *value == '\n') && 
            *value != 0)
        {
            *value = 0;
            value++;
        }

        if(*value == 0)
        {
// no value given
            continue;
        }

// delete the newline
        char *end = value;
        while(*end != '\n' && 
            *end != 0)
        {
            end++;
        }
        
        if(*end == '\n')
        {
            *end = 0;
        }
        
        printf("load_defaults %d key='%s' value='%s'\n", __LINE__, key, value);
        if(!strcasecmp(key, "PAN"))
        {
            start_pan = pan = atof(value);
        }
        else
        if(!strcasecmp(key, "TILT"))
        {
            start_tilt = tilt = atof(value);
        }
        else
        if(!strcasecmp(key, "PAN_SIGN"))
        {
            pan_sign = atoi(value);
        }
        else
        if(!strcasecmp(key, "TILT_SIGN"))
        {
            tilt_sign = atoi(value);
        }        
        else
        if(!strcasecmp(key, "LENS"))
        {
            lens = atoi(value);
        }        
        else
        if(!strcasecmp(key, "LANDSCAPE"))
        {
            landscape = atoi(value);
        }
        else
        if(!strcasecmp(key, "TRUCK"))
        {
            is_truck = atoi(value);
        }
        else
        if(!strcasecmp(key, "DEADBAND"))
        {
            deadband = atoi(value);
        }
        else
        if(!strcasecmp(key, "SPEED"))
        {
            speed = atoi(value);
        }
    }

    fclose(fd);
}

void save_defaults()
{
// HOME not available in /etc/rc.local
//    char *home = getenv("HOME");
    char *home = "/root";
    char string[TEXTLEN];
    sprintf(string, "%s/.tracker.rc", home);
    FILE *fd = fopen(string, "w");

    if(!fd)
    {
        printf("save_defaults %d: Couldn't open %s for writing\n", 
            __LINE__, 
            string);
        return;
    }

    fprintf(fd, "PAN %d\n", (int)start_pan);
    fprintf(fd, "TILT %d\n", (int)start_tilt);
    fprintf(fd, "PAN_SIGN %d\n", pan_sign);
    fprintf(fd, "TILT_SIGN %d\n", tilt_sign);
    fprintf(fd, "LENS %d\n", lens);
    fprintf(fd, "LANDSCAPE %d\n", landscape);
    fprintf(fd, "########################################\n");
    fprintf(fd, "TRUCK %d\n", is_truck);
    fprintf(fd, "DEADBAND %d\n", deadband);
    fprintf(fd, "SPEED %d\n", speed);

    fclose(fd);
}

void dump_settings()
{
    printf("dump_settings %d\n", __LINE__);
    printf("TRUCK %d\n", is_truck);
    printf("DEADBAND %d\n", deadband);
    printf("SPEED %d\n", speed);
}


void reset_zone(zone_t *zone, int *parts)
{
    zone->min_y = 0;
    zone->max_y = 0;
    zone->total = 0;
    zone->parts = parts;
}

int calculate_error(int current_x, int want_x)
{
    return current_x - want_x;
}


void update_zone(zone_t *zone, int body_part, int x, int y)
{
    int *part = zone->parts;
    while(*part >= 0)
    {
        if(*part == body_part)
        {
            zone->total++;

            if(zone->total == 1)
            {
                zone->min_y = zone->max_y = y;
            }
            else
            {
                zone->min_y = MIN(zone->min_y, y);
                zone->max_y = MAX(zone->max_y, y);
            }
            break;
        }
        part++;
    }
}


void do_feedback(double delta, 
    op::Array<float> &poseKeypoints,
    int w, 
    int h)
{
    int animals = poseKeypoints.getSize(0);
    if(animals > MAX_ANIMALS)
    {
        animals = MAX_ANIMALS;
    }
    
    if(is_truck)
    {
        if(animals > 1)
            animals = 1;
    }

// printf("do_feedback %d animals=%d w=%d h=%d\n", 
// __LINE__, 
// animals,
// w,
// h);


// focal point of all animals
    float total_x = 0;
    float total_y = 0;
    int center_x = w / 2;
    int center_y = h / 2;
    int largest = 0;
    zone_t zones[TOTAL_ZONES];
// size of highest head
    int head_size = 0;
    bzero(zones, sizeof(zone_t) * TOTAL_ZONES);

// fuse all bodies
    for(int human = 0; human < animals; human++)
    {
        body_t *body = &bodies[human];

// average center of all bodies
        total_x += (body->x1 + body->x2) / 2;
        total_y += (body->y1 + body->y2) / 2;


// get minimum head Y & size of head with minimum Y
        if(body->zones[HEAD_ZONE].total &&
            (!zones[HEAD_ZONE].total ||
                body->zones[HEAD_ZONE].min_y < zones[HEAD_ZONE].min_y))
        {
            head_size = body->head_size;
            zones[HEAD_ZONE].min_y = body->zones[HEAD_ZONE].min_y;
        }


// accumulate total of all zones
        for(int i = 0; i < TOTAL_ZONES; i++)
        {
            zones[i].total += body->zones[i].total;
        }
    }


    if(animals > 0)
    {
// average coords of all bodies
        total_x /= animals;
        total_y /= animals;

        if(is_truck)
        {
            int x_error = TO_PERCENT_X(calculate_error(total_x, 
                center_x));
            if(x_error >= deadband)
            {
                adc = MIN_LEFT +
                    (MAX_LEFT - MIN_LEFT) *
                    (x_error - deadband) *
                    speed / 
                    100 /
                    100;
                CLAMP(adc, 0, 255);
            }
            else
            if(x_error < -deadband)
            {
                adc = MIN_RIGHT - 
                    (MIN_RIGHT - MAX_RIGHT) *
                    (-deadband - x_error) * 
                    speed / 
                    100 /
                    100;
                CLAMP(adc, 0, 255);
            }
            else
            {
// stop servo
                adc = ADC_CENTER;
            }

//printf("do_feedback %d adc=%d\n", __LINE__, adc);
        }
        else
        {
// track horizontal avg of all bodies
            int x_error = calculate_error(total_x, 
                center_x);
            if(TO_PERCENT_X(abs(x_error)) > lenses[lens].deadband)
            {
                float pan_change = delta * 
                    lenses[lens].x_gain * 
                    TO_PERCENT_X(x_error);
                CLAMP(pan_change, 
                    -lenses[lens].max_pan_change, 
                    lenses[lens].max_pan_change);
                pan += pan_change * pan_sign;

    // printf("do_feedback %d delta=%f x_error=%d percent=%d pan_change=%f\n", 
    // __LINE__, 
    // delta,
    // x_error, 
    // TO_PERCENT_X(x_error), 
    // pan_change);
            }


#ifdef TRACK_TILT
            int top_y = FROM_PERCENT_Y(lenses[lens].top_y);
// // range of top_y based on head size
//                     int top_y1 = FROM_PERCENT_Y(TOP_Y1);
//                     int top_y = top_y1;
//                     int top_y2 = FROM_PERCENT_Y(TOP_Y2);
// 
// // range of head size
//                     int head_size1 = FROM_PERCENT_Y(HEAD_SIZE1);
//                     int head_size2 = FROM_PERCENT_Y(HEAD_SIZE2);
// 
//                     if(head_size >= head_size1 && head_size < head_size2)
//                     {
//                         double fraction = (double)(head_size - head_size1) / 
//                             (head_size2 - head_size1);
//                         top_y = (int)(fraction * top_y2 + 
//                             (1.0 - fraction) * top_y1);
//                     }
//                     else
//                     if(head_size >= head_size2)
//                     {
//                         top_y = top_y2;
//                     }
// printf("Process: workConsumer %d: head_size=%d top_y=%d\n", 
// __LINE__, 
// TO_PERCENT_Y(head_size),
// top_y);



            int y_error = 0;
            float tilt_change = 0;
    // head & foot zones visible.  Track the center or the head.
            if(zones[HEAD_ZONE].total > 0 &&
                zones[FOOT_ZONE].total > 0)
            {
    // head is too high.  track the head
                if(zones[HEAD_ZONE].min_y < top_y)
                {
                    y_error = calculate_error(zones[HEAD_ZONE].min_y, 
                        top_y);
                }
                else
    // track the center of the body
                {
                    y_error = calculate_error(total_y, 
                        center_y);
                }

            }
            else
    // head is visible but feet are hidden.  Track the head.
            if(zones[HEAD_ZONE].total > 0)
            {
    // assume the body is too big to fit in frame, 
    // so track the minimum y of any head.
    //                        if(biggest_h > height / 2)
                {
                    y_error = calculate_error(zones[HEAD_ZONE].min_y, 
                        top_y);
                }
    //                         else
    //                         {
    // // assume the body is obstructed by the foreground but would fit in frame, 
    // // so center it.  This is vulnerable to glitches if the body is too close.
    //                             y_error = calculate_error(total_y, 
    //                                 center_y);
    //                         }
            }
    // other zones are visible but head is hidden.  Tilt up.
            else
            if(zones[NECK_ZONE].total > 0 ||
                zones[HIP_ZONE].total > 0 ||
                zones[FOOT_ZONE].total > 0)
            {
                y_error = -FROM_PERCENT_Y(lenses[lens].tilt_search);
            }

    // apply the Y error to the servo
            if(abs(y_error) > FROM_PERCENT_Y(lenses[lens].deadband))
            {
                tilt_change = delta * 
                    lenses[lens].y_gain * 
                    TO_PERCENT_Y(y_error);
                CLAMP(tilt_change, 
                    -lenses[lens].max_tilt_change, 
                    lenses[lens].max_tilt_change);
                tilt -= tilt_change * tilt_sign;
            }
#endif // TRACK_TILT
        } // !is_truck

//printf("pan_change=%d tilt_change=%d\n", (int)pan_change, (int)tilt_change);
//    printf("pan=%d tilt=%d\n", (int)(pan - start_pan), (int)(tilt - start_tilt));

        write_servos(0);
    }
}

int probe_usb(uint32_t vid, uint32_t pid)
{
	struct libusb_device_handle *devh = libusb_open_device_with_vid_pid(0, vid, pid);
	if(devh)
	{
		libusb_close(devh);
		return 1;
	}
    return 0;
}

#define CLEAR_CAM_ERRORS \
error_flags &= ~(CAM_ENUM_ERROR | DEV_VIDEO_ERROR | CAM_STARTING_ERROR | VIDEO_IOCTL_ERROR);

// probe for the video device
int open_hdmi(int verbose)
{
    std::vector<char*> paths;
    char string[TEXTLEN];
    FILE *fd;
    int got_usb = 0;

// may have USB without paths or paths with broken USB probing.
// discover the USB devices
//     libusb_device **devs;
//     int total = libusb_get_device_list(0, &devs);
//     for(int i = 0; i < total; i++)
//     {
//         libusb_device_descriptor desc;
//         libusb_get_device_descriptor(devs[i], &desc);
//         printf("open_hdmi %d %04x %04x\n", 
//             __LINE__, 
//             desc.idVendor, 
//             desc.idProduct);
//         if(desc.idVendor == WEBCAM_VID1 && desc.idProduct == WEBCAM_PID1 ||
//             desc.idVendor == WEBCAM_VID2 && desc.idProduct == WEBCAM_PID2 ||
//             desc.idVendor == HDMI_VID && desc.idProduct == HDMI_PID)
//         {
//             got_usb = 1;
//             break;
//         }
//     }
//     libusb_free_device_list(devs, 1);

// may fail this while having /dev/video
    if(probe_usb(WEBCAM_VID1, WEBCAM_PID1) ||
        probe_usb(WEBCAM_VID2, WEBCAM_PID2) ||
        probe_usb(HDMI_VID, HDMI_PID))
    {
        got_usb = 1;
    }



// hangs
//     fd = popen("lsusb", "r");
//     while(!feof(fd))
//     {
//         char *result = fgets(string, TEXTLEN, fd);
//         if(!result)
//         {
//             break;
//         }
//         if(strstr(result, WEBCAM_ID1) ||
//             strstr(result, WEBCAM_ID2) ||
//             strstr(result, HDMI_ID))
//         {
//             got_usb = 1;
//             break;
//         }
//     }
//     fclose(fd);

// discover the paths
    fd = popen("ls /dev/video*", "r");
    while(!feof(fd))
    {
        char *result = fgets(string, TEXTLEN, fd);
        if(!result)
        {
            break;
        }
// strip the newlines
        while(strlen(result) > 1 &&
            result[strlen(result) - 1] == '\n')
        {
            result[strlen(result) - 1] = 0;
        }
        paths.push_back(strdup(result));
    }
    fclose(fd);

    if(paths.size() == 0)
    {
        if(!got_usb)
        {
            printf("open_hdmi %d: no USB device\n", __LINE__);
            CLEAR_CAM_ERRORS
            error_flags |= CAM_ENUM_ERROR;
            send_error();
            sleep(1);
            return -1;
        }
        else
        if((error_flags & CAM_ENUM_ERROR))
        {
            printf("open_hdmi %d: USB device found\n", __LINE__);
            CLEAR_CAM_ERRORS
            send_error();
        }


        printf("open_hdmi %d: no video device\n", __LINE__);
        CLEAR_CAM_ERRORS
        error_flags |= DEV_VIDEO_ERROR;
        send_error();
        sleep(1);
        return -1;
    }
    else
    {
        printf("open_hdmi %d: video device found\n", __LINE__);
        CLEAR_CAM_ERRORS
        send_error();
    }

    int fd2 = -1;
    for(int i = 0; i < paths.size(); i++)
    {
        printf("open_hdmi %d: opening %s\n", __LINE__, paths.at(i));
        CLEAR_CAM_ERRORS
        error_flags |= CAM_STARTING_ERROR;
        send_error();


        fd2 = open(paths.at(i), O_RDWR);
        if(fd2 < 0)
        {
            continue;
        }

        struct v4l2_buffer buffer;
        struct v4l2_requestbuffers requestbuffers;
        requestbuffers.count = VIDEO_BUFFERS;
        requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        requestbuffers.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd2, VIDIOC_REQBUFS, &requestbuffers) < 0)
        {
            printf("open_hdmi %d: VIDIOC_REQBUFS failed\n",
                __LINE__);
            close(fd2);
            fd2 = -1;
        }
        else
        {
            for(int j = 0; j < VIDEO_BUFFERS; j++)
            {
                buffer.type = requestbuffers.type;
                buffer.index = j;

                if(ioctl(fd2, VIDIOC_QUERYBUF, &buffer) < 0)
				{
					printf("open_hdmi %d: VIDIOC_QUERYBUF failed\n",
                        __LINE__);
                    close(fd2);
                    fd2 = -1;
                    break;
				}
                else
                {
                    mmap_buffer[j] = (unsigned char*)mmap(NULL,
					    buffer.length,
					    PROT_READ | PROT_WRITE,
					    MAP_SHARED,
					    fd2,
					    buffer.m.offset);
                    printf("open_hdmi %d: allocated buffer size=%d\n",
                        __LINE__,
                        buffer.length);
                    if(ioctl(fd2, VIDIOC_QBUF, &buffer) < 0)
                    {
                        printf("open_hdmi %d: VIDIOC_QBUF failed\n",
                            __LINE__);
                        close(fd2);
                        fd2 = -1;
                        break;
                    }
                }
            }
        }

        if(fd2 >= 0)
        {
            int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            printf("open_hdmi %d: starting\n", __LINE__);
	        if(ioctl(fd2, VIDIOC_STREAMON, &streamon_arg) < 0)
            {
		        printf("open_hdmi %d: VIDIOC_STREAMON failed\n",
                    __LINE__);
                for(int j = 0; j < VIDEO_BUFFERS; j++)
                {
                    munmap(mmap_buffer[j], buffer.length);
                }
                close(fd2);
                fd2 = -1;
            }
        }
        break;
    }


// delete the paths
    while(paths.size() > 0)
    {
        free(paths.back());
        paths.pop_back();
    }


    if(fd2 < 0)
    {
        printf("do_tracker %d: open/config error\n", __LINE__);
        CLEAR_CAM_ERRORS
        error_flags |= VIDEO_IOCTL_ERROR;
        send_error();
        sleep(1);
        return -1;
    }
    else
    {
        printf("do_tracker %d: video configured\n", __LINE__);
        CLEAR_CAM_ERRORS
        send_error();
    }

    return fd2;
}


static int compare_height(const void *ptr1, const void *ptr2)
{
    body_t *item1 = (body_t*)ptr1;
    body_t *item2 = (body_t*)ptr2;
    return item1->y1 > item2->y1;
}

void do_tracker()
{
    cudaSetDevice(0);

    landscape_engine.load(ENGINE_LANDSCAPE, 
        CAM_W, 
        CAM_H, 
        input_w, 
        input_h);

    if(!is_truck)
        portrait_engine.load(ENGINE_PORTRAIT, 
            CAM_H, 
            CAM_H * 3 / 2, 
            input_w, 
            input_h);





// YUV to RGB conversion from guicast
    cmodel_init();

    if(is_truck)
    {
// YUV planar 1280x720 from JPEGs
        for(int i = 0; i < INPUT_IMAGES; i++)
        {
            hdmi_image[i] = new uint8_t[input_w * input_h * 3 / 2];
        }
    }
    else
    {
    #ifdef RAW_HDMI
    // YUV packed 640x480 direct from the HDMI converter
        for(int i = 0; i < INPUT_IMAGES; i++)
        {
            hdmi_image[i] = new uint8_t[input_w * input_h * 2];
            hdmi_rows[i] = new uint8_t*[input_h];
            for(int j = 0; j < input_h; j++)
            {
                hdmi_rows[i][j] = hdmi_image[i] + j * input_w * 2;
            }
        }

    #else // RAW_HDMI
    // RGB 1920x1080 from JPEGs
    // scaling table
        int x_lookup[INPUT_W];
        for(int i = 0; i < INPUT_W; i++)
            x_lookup[i] = i * HDMI_W / CAM_W * 3;
    // destination for JPEG decompression
        hdmi_image = new uint8_t[HDMI_W * 3 * HDMI_H];
        hdmi_rows = new uint8_t*[HDMI_H];
        for(int i = 0; i < HDMI_H; i++)
        {
            hdmi_rows[i] = hdmi_image + HDMI_W * i * 3;
        }
    #endif // !RAW_HDMI
    }


    float fps = 0;
    struct timespec fps_time1;
    struct timespec feedback_time1;
    clock_gettime(CLOCK_MONOTONIC, &fps_time1);
    clock_gettime(CLOCK_MONOTONIC, &feedback_time1);
    int frame_count = 0;

    int current_device = DEVICE0;
    int cam_fd = -1;

    int verbose = 1;
    while(1)
    {
        if(cam_fd < 0)
        {
            cam_fd = open_hdmi(verbose);
        }

        if(cam_fd >= 0)
        {
            struct v4l2_buffer buffer;
		    bzero(&buffer, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buffer.memory = V4L2_MEMORY_MMAP;
            if(ioctl(cam_fd, VIDIOC_DQBUF, &buffer) < 0)
            {
                printf("do_tracker %d: VIDIOC_DQBUF failed\n",
                    __LINE__);
                if((error_flags & VIDEO_IOCTL_ERROR) == 0)
                {
                    error_flags |= VIDEO_IOCTL_ERROR;
                    send_error();
                }
                close(cam_fd);
                cam_fd = -1;
                sleep(1);
            }
            else
            {
                if((error_flags & VIDEO_IOCTL_ERROR))
                {
                    printf("do_tracker %d: VIDIOC_DQBUF success\n",
                        __LINE__);
                    error_flags &= ~VIDEO_IOCTL_ERROR;
                    send_error();
                }
// printf("do_tracker %d picture_size=%d picture_data=%p %02x %02x %02x %02x %02x %02x %02x %02x\n", 
// __LINE__, 
// buffer.bytesused, 
// mmap_buffer[buffer.index],
// mmap_buffer[buffer.index][0],
// mmap_buffer[buffer.index][1],
// mmap_buffer[buffer.index][2],
// mmap_buffer[buffer.index][3], 
// mmap_buffer[buffer.index][4],
// mmap_buffer[buffer.index][5],
// mmap_buffer[buffer.index][6],
// mmap_buffer[buffer.index][7]);
// FILE *fd = fopen("/tmp/test.jpg", "w");
// fwrite(mmap_buffer[buffer.index], 1, buffer.bytesused, fd);
// fclose(fd);

                if(is_truck)
                {
// always JPEG
                    int decoded_w;
                    int decoded_h;
                    decompress_jpeg_yuv(mmap_buffer[buffer.index], 
                        buffer.bytesused,
                        &decoded_w,
                        &decoded_h,
                        hdmi_image[current_input]);

// these have to be reset for every frame
// reset saturation
                    struct v4l2_control ctrl_arg;
                    ctrl_arg.id = 0x980902;
                    ctrl_arg.value = 127;
                    ioctl(cam_fd, VIDIOC_S_CTRL, &ctrl_arg);
// reset backlight compensation
                    ctrl_arg.id = 0x98091c;
                    ctrl_arg.value = 0;
                    ioctl(cam_fd, VIDIOC_S_CTRL, &ctrl_arg);
// release the buffer
                    ioctl(cam_fd, VIDIOC_QBUF, &buffer);
                }
                else
                {

                #ifndef RAW_HDMI
                    decompress_jpeg(mmap_buffer[buffer.index], buffer.bytesused);
    // release the buffer
                    ioctl(fd, VIDIOC_QBUF, &buffer);
    // scale the HDMI image to the INPUT size
                    for(int i = 0; i < INPUT_H; i++)
                    {
                        uint8_t *dst = input_rows_l[current_input][i];
                        uint8_t *src = hdmi_rows[i * HDMI_H / INPUT_H];
                        for(int j = 0; j < INPUT_W; j++)
                        {
                            uint8_t *src2 = src + x_lookup[j];
                            *dst++ = *src2++;
                            *dst++ = *src2++;
                            *dst++ = *src2++;
                        }
                    }
                #else // !RAW_HDMI

                    memcpy(hdmi_image[current_input], 
                        mmap_buffer[buffer.index], 
                        HDMI_W * HDMI_H * 2);
    // release the buffer
                    ioctl(cam_fd, VIDIOC_QBUF, &buffer);
                #endif // RAW_HDMI
                }



// The pose tracking
                Engine *engine;
                if(landscape || is_truck)
                    engine = &landscape_engine;
                else
                    engine = &portrait_engine;

                if(is_truck)
                    engine->process(YUV_PLANAR, hdmi_image[current_input]);
                else
                {
                #ifdef RAW_HDMI
                    engine->process(YUV_PACKED, hdmi_image[current_input]);
                #else
                    engine->process(YUV_NONE, hdmi_image[current_input]);
                #endif
                }


// sort by size
                int orig_animals = engine->mPoseKeypoints.getSize(0);
                for (int animal = 0; animal < orig_animals; animal++)
                {
                    const auto& bodyParts = engine->mPoseKeypoints.getSize(1);
                    body_t *body = &bodies[animal];
                    body->orig_animal = animal;

// reset the zones
                    for(int i = 0; i < TOTAL_ZONES; i++)
                    {
                        reset_zone(&body->zones[i], zone_parts[i]);
                    }


                    if(bodyParts >= BODY_PARTS)
                    {
            // get bounding box of all body parts for now
                        int have_body_part = 0;
                        for(int body_part = 0; body_part < BODY_PARTS; body_part++)
                        {
                            int x = (int)engine->mPoseKeypoints[{ animal, body_part, 0 }];
                            int y = (int)engine->mPoseKeypoints[{ animal, body_part, 1 }];

            // body part was detected
                            if(x > 0 && y > 0)
                            {
            // initialize the coords
                                if(!have_body_part)
                                {
                                    body->x1 = body->x2 = x;
                                    body->y1 = body->y2 = y;
                                    have_body_part = 1;
                                }
                                else
                                {
            // get extents of coords
                                    if(body->x1 > x)
                                    {
                                        body->x1 = x;
                                    }

                                    if(body->x2 < x)
                                    {
                                        body->x2 = x;
                                    }

                                    if(body->y1 > y)
                                    {
                                        body->y1 = y;
                                    }

                                    if(body->y2 < y)
                                    {
                                        body->y2 = y;
                                    }
                                }


            // update the zone this body part belongs to
                                for(int i = 0; i < TOTAL_ZONES; i++)
                                {
                                    update_zone(&body->zones[i], body_part, x, y);
                                }
                            }
                        }


            // get the head size from the head zones
                        if(body->zones[NECK_ZONE].total &&
                            body->zones[HEAD_ZONE].total)
                        {
            // expand head by 1/3 since openpose only detects eyes
            // assume the head is vertical
                            int h = body->zones[NECK_ZONE].max_y -
                                body->zones[HEAD_ZONE].min_y;
                            body->y1 -= h / 2;
                            body->zones[HEAD_ZONE].min_y -= h / 2;

                            body->head_size = body->zones[NECK_ZONE].max_y -
                                body->zones[HEAD_ZONE].min_y;
                        }


            // debug
            //                     for(int i = 0; i < TOTAL_ZONES; i++)
            //                     {
            //                         if(body->zones[i].total)
            //                         {
            //                             printf("zone %d: y=%d-%d total=%d ", 
            //                                 i,
            //                                 body->zones[i].min_y, 
            //                                 body->zones[i].max_y, 
            //                                 body->zones[i].total);
            //                         }
            //                     }
            //                     printf("\n");
                    }
                }

                
                
                qsort(bodies, orig_animals, sizeof(body_t), compare_height);
// for(int i = 0; i < orig_animals; i++)
// {
// printf("do_tracker %d i=%d y1=%d\n", __LINE__, i, bodies[i].y1);
// }


// draw the vijeo
#ifndef USE_SERVER
                int dst_x;
                int dst_y;
                int dst_w;
                int dst_h;

                if(current_operation == CONFIGURING ||
                    current_operation == STARTUP)
                {
    // show video during configuration
                    if(INPUT_H > INPUT_W)
                    {
                        dst_h = WINDOW_H;
                        dst_w = dst_h * INPUT_W / INPUT_H;
                        dst_x = WINDOW_W - gui->reserved_w / 2 - dst_w / 2;
                        dst_y = 0;
                    }
                    else
                    {
                        dst_w = gui->reserved_w;
                        dst_h = dst_w * INPUT_H / INPUT_W;
                        dst_x = WINDOW_W - dst_w;
                        dst_y = WINDOW_H / 2 - dst_h / 2;
                    }
                }
                else
                {
    // show video during tracking
                    if(INPUT_H > INPUT_W)
                    {
                        dst_h = WINDOW_H;
                        dst_w = dst_h * INPUT_W / INPUT_H;
                    }
                    else
                    {
                        dst_w = WINDOW_W;
                        dst_h = dst_w * INPUT_H / INPUT_W;
                    }

                    dst_x = WINDOW_W / 2 - dst_w / 2;
                    dst_y = 0;
                }

                draw_video(hdmi_image[current_input], 
                    dst_x,
                    dst_y,
                    dst_w, 
                    dst_h,
                    INPUT_W,
                    INPUT_H,
                    INPUT_W * 3);
// send output frame to server

// Xlib conflicts with openpose so translate array
                for(int i = 0; i < mPoseKeypoints.getSize(0); i++)
                {
                    int xy_array[BODY_PARTS * 2];
                    for(int j = 0; j < BODY_PARTS; j++)
                    {
                        xy_array[j * 2 + 0] = (int)mPoseKeypoints[{ i, j, 0 }];
                        xy_array[j * 2 + 1] = (int)mPoseKeypoints[{ i, j, 1 }];
                    }
//                    draw_body(xy_array, INPUT_W, INPUT_H);
                }
#else // !USE_SERVER


// server is ready for data
                if(current_input2 < 0)
                {
// send keypoints
                    int animals = engine->mPoseKeypoints.getSize(0);
//printf("do_tracker %d animals=%d\n", __LINE__, animals);

                    if(animals > MAX_ANIMALS) animals = MAX_ANIMALS;
                    if(is_truck && animals > 1)
                        animals = 1;
                    
                    
                    int offset = HEADER_SIZE;
                    int max_x = 0;
                    int max_y = 0;

                    vijeo_buffer[offset++] = (int)(fps * 256) & 0xff;
                    vijeo_buffer[offset++] = (int)fps;

                    vijeo_buffer[offset++] = animals;
                    vijeo_buffer[offset++] = 0;
                    for(int i = 0; i < animals; i++)
                    {
                        int orig_index = bodies[i].orig_animal;
                        for(int j = 0; j < BODY_PARTS; j++)
                        {
                            int x = (int)engine->mPoseKeypoints[{ orig_index, j, 0 }];
                            int y = (int)engine->mPoseKeypoints[{ orig_index, j, 1 }];
                            vijeo_buffer[offset++] = x & 0xff;
                            vijeo_buffer[offset++] = (x >> 8) & 0xff;
                            vijeo_buffer[offset++] = y & 0xff;
                            vijeo_buffer[offset++] = (y >> 8) & 0xff;
                            if(x > max_x) max_x = x;
                            if(y > max_y) max_y = y;
                        }
                    }
                    send_vijeo(current_input, offset - HEADER_SIZE);
// avoid a race condition by not toggling this if the server is busy
                    current_input = !current_input;
// printf("do_tracker %d animals=%d max_x=%d max_y=%d\n", 
// __LINE__, 
// animals,
// max_x,
// max_y);
                }

#endif // USE_SERVER



                frame_count++;
                struct timespec fps_time2;
                clock_gettime(CLOCK_MONOTONIC, &fps_time2);
                int64_t diff = TO_MS(fps_time2) - TO_MS(fps_time1);
                if(diff >= 1000)
                {
                    fps = (double)frame_count * 1000 / diff;
                    printf("do_tracker %d: FPS: %f\n",
                        __LINE__,
                        fps);
                    frame_count = 0;
                    fps_time1 = fps_time2;
                }
                
                
                struct timespec feedback_time2;
                clock_gettime(CLOCK_MONOTONIC, &feedback_time2);
// fractional seconds
                double delta = (double)(TO_MS(feedback_time2) -
                    TO_MS(feedback_time1)) / 
                    1000;
                feedback_time1 = feedback_time2;
                if(current_operation == TRACKING)
                {
                    do_feedback(delta, 
                        engine->mPoseKeypoints,
                        engine->cam_w,
                        engine->cam_h);
                }
                else
                if(is_truck && adc != ADC_CENTER)
                {
                    adc = ADC_CENTER;
                    write_servos(0);
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    load_defaults();
    dump_settings();
    libusb_init(0);

    if(is_truck)
    {
        input_w = TRUCK_W;
        input_h = TRUCK_H;
    }
    else
    {
        input_w = HDMI_W;
        input_h = HDMI_H;
    }

#ifdef USE_SERVER
    init_server();
#else
    init_gui();
#endif

    int result = init_servos();

#ifndef USE_SERVER
    draw_startup();
#endif // USE_SERVER

    do_tracker();
}












