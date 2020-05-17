/*
 * tracking camera
 * Copyright (C) 2019-2020 Adam Williams <broadcast at earthling dot net>
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

// this runs on a laptop to do the machine vision

// be sure to install the FTDI driver for Macos, 
// make sure the virtual machine isn't bound to it & 
// the right webcam is being selected

// To build it:
// 
// ./make.sh tracker
// 
// To run it:
// 
// ./tracker.sh
// 
// install kernel module on Linux:
// insmod /lib/modules/4.9.39/kernel/drivers/video/nvidia-uvm.ko


// the earlier trackers using LIDAR & difference keying
// are in the gimbal directory as motion.c cam_lidar.c cam.c
// the servo driver is in the servos/ directory

#include <openpose/headers.hpp>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>

#include "guicast.h"
#include "keys.h"
#include "mutex.h"

// load frames from test_input
//#define LOAD_TEST_INPUT
// load frames from DSLR
#define LOAD_GPHOTO2
// use a webcam as input
//#define LOAD_WEBCAM

// save openpose output in test_output
#define SAVE_OUTPUT
// output photos go here
#define OUTPATH "test_output"

// tilt tracking is optional
#define TRACK_TILT

// maximum humans to track
#define MAX_HUMANS 1

// PWM limits
#define MIN_PWM 1000
#define MAX_PWM 32700

// PWM limits beyond starting point for tracking
#define TILT_MAG 1500
#define PAN_MAG 5000

#ifdef __clang__
#define MODELS "../openpose.mac/models/"
#else
#define MODELS "../openpose/models/"
#endif

#define TEXTLEN 1024
#define BODY_PARTS 25

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define TO_MS(x) ((x).tv_sec * 1000 + (x).tv_usec / 1000)

// different parameters for different lenses
#define LENS_15 0
#define LENS_28 1
#define LENS_50 2
#define TOTAL_LENSES 3

typedef struct
{
// manual step
    int pan_step;
    int tilt_step;
// PID gain
    float x_gain;
    float y_gain;
// Limit of PWM changes
    int max_tilt_change;
    int max_pan_change;
// Y error when head is above frame
    int tilt_search;
} lens_t;

lens_t lenses[] = 
{
    { 100, 100, 4, 4, 50, 50, 200 }, // 15mm
    { 100, 100, 2, 2, 50, 50, 150 }, // 28mm
    { 50, 50, 1, 1, 50, 50, 100 }, // 50mm
};



// normalize the error pixels to the EOS RP preview frame sizes
#define NORMALIZE_DISTANCE(error) (error * 576 / height)

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
    int x1, y1, x2, y2;
// last detected size of the head for tilt tracking
    int head_size;
    zone_t zones[TOTAL_ZONES];
} body_t;

static body_t bodies[MAX_HUMANS];

static float pan = 18676;
static float tilt = 21076;
static float start_pan = pan;
static float start_tilt = tilt;
static int pan_sign = 1;
static int tilt_sign = 1;
static int lens = LENS_15;

static int servo_fd = -1;
static int frames = 0;

#define STARTUP 0
#define CONFIGURING 1
#define TRACKING 2
static int current_operation = STARTUP;
static int have_time1 = 0;


#define WINDOW_W 1920
#define WINDOW_H 1040
#define MARGIN 10



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
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}


int init_servos()
{
#ifdef __clang__
    servo_fd = init_serial("/dev/cu.usbserial-AL03OO1F");
#else
	servo_fd = init_serial("/dev/ttyUSB0");
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyUSB1");
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyUSB2");
#endif


    if(servo_fd >= 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void write_servos(int use_pwm_limits)
{
	if(servo_fd >= 0)
	{
#define SYNC_CODE 0xe5
#define BUFFER_SIZE 5

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
        buffer[0] = SYNC_CODE;
        buffer[1] = pan_i;
        buffer[2] = pan_i >> 8;
        buffer[3] = tilt_i;
        buffer[4] = tilt_i >> 8;

//printf("write_servos %d %d %d\n", __LINE__, pan_i,  tilt_i);
		int temp = write(servo_fd, buffer, BUFFER_SIZE);
	}
}



void load_defaults()
{
    char *home = getenv("HOME");
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
            pan = atof(value);
        }
        else
        if(!strcasecmp(key, "TILT"))
        {
            tilt = atof(value);
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
    }
    
    fclose(fd);
}

void save_defaults()
{
    char *home = getenv("HOME");
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
    
    fprintf(fd, "PAN %d\n", (int)pan);
    fprintf(fd, "TILT %d\n", (int)tilt);
    fprintf(fd, "PAN_SIGN %d\n", pan_sign);
    fprintf(fd, "TILT_SIGN %d\n", tilt_sign);
    fprintf(fd, "LENS %d\n", lens);
    
    fclose(fd);
}

const char* lens_to_text(int lens)
{
    switch(lens)
    {
        case LENS_15:
            return "15mm";
            break;
        case LENS_28:
            return "28mm";
            break;
        case LENS_50:
            return "50mm";
            break;
        default:
            return "Unknown";
    }
}

class GUI : public BC_Window
{
public:
    GUI() : BC_Window("Tracker",
        0, // x
		0, // y
		WINDOW_W, // w
		WINDOW_H, // h
		-1, // minw
		-1, // minh
		0, // allow_resize
		0, // private_color
		1, // hide
        BLACK) // bg_color
    {
    };

	int close_event()
	{
		set_done(0);
		return 1;
	};
    
    int keypress_event()
    {
//        printf("GUI::keypress_event %d %c\n", __LINE__, get_keypress());
        int need_print_values = 0;
        int need_write_servos = 0;
        switch(get_keypress())
        {
            case ESC:
            case 'q':
                set_done(0);
                return 1;
                break;
            
            case ' ':
            case RETURN:
// advance operation
                if(current_operation == STARTUP)
                {
                    current_operation = CONFIGURING;
                    clear_box(0, 0, WINDOW_W, WINDOW_H, 0);
                    int text_h = get_text_height(LARGEFONT, "q0");
                    int y = text_h + MARGIN;
                    int x = MARGIN;
                    set_color(WHITE);
                    char string[BCTEXTLEN];
                    sprintf(string,
                        "Press keys to aim the mount.\n\n"
                        "PWM Values should be as close to 0 as possible.\n"
                        "a - left   d - right   w - up   s - down\n"
                        "t - invert tilt sign   p - invert pan sign\n"
                        "l - change lens\n"
                        "SPACE or ENTER to save defaults & begin tracking\n"
                        "ESC to give up & go to a movie.");
                    draw_text(x, 
                        y, 
                        string);
                    flash(1);
                    
// write it a few times to defeat UART initialization glitches
                    write_servos(1);
                    usleep(100000);
                    write_servos(1);
                    usleep(100000);
                    write_servos(1);
                    usleep(100000);
                    write_servos(1);
                    
                    need_print_values = 1;
                }
                else
                if(current_operation == CONFIGURING)
                {
                    start_pan = pan;
                    start_tilt = tilt;
                    ::save_defaults();
                    current_operation = TRACKING;
                    clear_box(0, 0, WINDOW_W, WINDOW_H, 0);
                    flash(1);
                }
                break;
            
            case 'w':
                tilt += lenses[lens].tilt_step * tilt_sign;
                need_write_servos = 1;
                need_print_values = 1;
                break;
            
            case 's':
                tilt -= lenses[lens].tilt_step * tilt_sign;
                need_write_servos = 1;
                need_print_values = 1;
                break;
            
            case 'a':
                pan -= lenses[lens].pan_step * pan_sign;
                need_write_servos = 1;
                need_print_values = 1;
                break;
            
            case 'd':
                pan += lenses[lens].pan_step * pan_sign;
                need_write_servos = 1;
                need_print_values = 1;
                break;
            
            case 't':
                tilt_sign *= -1;
                need_print_values = 1;
                break;
            
            case 'p':
                pan_sign *= -1;
                need_print_values = 1;
                break;
           
            case 'l':
                lens++;
                if(lens >= TOTAL_LENSES)
                {
                    lens = 0;
                }
                need_print_values = 1;
                break;
        }
        
        if(need_print_values && current_operation == CONFIGURING)
        {
            char string[BCTEXTLEN];
            sprintf(string, 
                "PAN=%d\nTILT=%d\nPAN_SIGN=%d\nTILT_SIGN=%d\nLENS=%s", 
                (int)(pan - (MAX_PWM + MIN_PWM) / 2),
                (int)(tilt - (MAX_PWM + MIN_PWM) / 2),
                pan_sign,
                tilt_sign,
                lens_to_text(lens));
            int line_h = get_text_height(LARGEFONT, "0");
            int text_h = get_text_height(LARGEFONT, string);
            int text_w = get_text_width(LARGEFONT, string);
            int text_x = WINDOW_W - MARGIN - text_w;
            int text_y = line_h + MARGIN;
            clear_box(text_x, text_y - line_h, text_w, text_h);
            set_color(WHITE);
            draw_text(text_x, 
                text_y, 
                string);
            flash(text_x, text_y - line_h, text_w, text_h, 1);
        }
        
        if(need_write_servos)
        {
            write_servos(1);
        }

// trap all of them
        return 1;
    }
    
};

static GUI *gui;

class GUIThread : public Thread
{
public:
    GUIThread() : Thread()
    {
    }

    void run()
    {
        gui->run_window();
        exit(0);
    }
};

static GUIThread gui_thread;
static BC_Bitmap *gui_bitmap = 0;


void init_gui()
{
    gui = new GUI();
    gui_bitmap = new BC_Bitmap(gui, 
	    WINDOW_W,
	    WINDOW_H,
	    BC_BGR8888,
	    1); // use_shm
    gui->start_video();
    gui->show_window();

    gui_thread.start();
}



// draw a frame on the GUI
void draw_video(unsigned char *src, 
    int dst_x,
    int dst_y,
    int dst_w,
    int dst_h,
    int src_w,
    int src_h,
    int src_rowspan)
{
    unsigned char **dst_rows = gui_bitmap->get_row_pointers();

// draw the video
    int nearest_x[dst_w];
    int nearest_y[dst_h];
    for(int i = 0; i < dst_w; i++)
    {
        nearest_x[i] = i * src_w / dst_w;
    }
    for(int i = 0; i < dst_h; i++)
    {
        nearest_y[i] = i * src_h / dst_h;
    }
    
    for(int i = 0; i < dst_h; i++)
    {
        unsigned char *src_row = src + nearest_y[i] * src_rowspan;
        unsigned char *dst_row = dst_rows[i];
        for(int j = 0; j < dst_w; j++)
        {
            int src_x = nearest_x[j];
            *dst_row++ = src_row[src_x * 3 + 0];
            *dst_row++ = src_row[src_x * 3 + 1];
            *dst_row++ = src_row[src_x * 3 + 2];
            dst_row++;
        }
    }    

    gui->lock_window();
    gui->draw_bitmap(gui_bitmap, 
	    1,
	    dst_x, // dst coords
	    dst_y,
	    dst_w,
	    dst_h,
	    0, // src coords
	    0,
	    dst_w,
	    dst_h);
    gui->unlock_window();
}








#ifdef LOAD_GPHOTO2
// read frames from the cam
class FrameInput : public op::WorkerProducer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
    static int initialized;
    static int frame_count;
    static FILE *fd;
#define BUFSIZE1 0x100
#define BUFSIZE2 0x400000
    static unsigned char reader_buffer1[BUFSIZE1];
    static unsigned char reader_buffer2[BUFSIZE2];
    static unsigned char reader_buffer3[BUFSIZE2];
    static int frame_size;
#define GET_START_CODE 0
#define GET_STOP_CODE 1
    static int reader_state;
    static const uint8_t start_code[4];
    static const uint8_t stop_code[2];
    static int code_offset;
    static int output_offset;
    static pthread_mutex_t frame_lock;
// FrameInput waits for this
    static sem_t frame_ready_sema;


    void initializationOnThread() {}

// read frames from gphoto2
    static void* reader_thread(void *ptr)
    {
        struct timeval time1;
        gettimeofday(&time1, 0);

        printf("FrameInput::reader_thread %d\n", __LINE__);
        while(1)
        {
            if(!fd)
            {
                printf("FrameInput::workProducer %d running gphoto2\n", __LINE__);
// Run gphoto2
                fd = popen("gphoto2 --stdout --capture-movie", "r");
                if(!fd)
                {
                    printf("FrameInput::workProducer %d: failed to run gphoto2\n",
                        __LINE__);
                    sleep(1);
                }
            }

            int bytes_read = 0;
            if(fd)
            {
                bytes_read = fread(reader_buffer1, 1, BUFSIZE1, fd);
                if(bytes_read <= 0)
                {
                    printf("FrameInput::reader_thread %d disconnected\n", __LINE__);
                    fclose(fd);
                    fd = 0;
                    sleep(1);
                }
            }
            
//             printf("FrameInput::reader_thread %d got %d bytes %02x %02x %02x %02x\n", 
//                 __LINE__, 
//                 bytes_read,
//                 reader_buffer1[0],
//                 reader_buffer1[1],
//                 reader_buffer1[2],
//                 reader_buffer1[3]);
            
            if(fd)
            {
                for(int i = 0; i < bytes_read; i++)
                {
                    uint8_t c = reader_buffer1[i];
                    if(reader_state == GET_START_CODE)
                    {
                        if(c == start_code[code_offset])
                        {
                            code_offset++;
                            if(code_offset >= sizeof(start_code))
                            {
                                memcpy(reader_buffer2, start_code, sizeof(start_code));
                                reader_state = GET_STOP_CODE;
                                output_offset = sizeof(start_code);
                            }
                        }
                        else
                        {
                            code_offset = 0;
                            if(c == start_code[code_offset])
                            {
                                code_offset++;
                            }
                        }
                    }
                    else
                    {
                        if(output_offset < BUFSIZE2 - sizeof(stop_code))
                        {
                            reader_buffer2[output_offset++] = c;
                        }

                        if(c == stop_code[code_offset])
                        {
                            code_offset++;
                            if(code_offset >= sizeof(stop_code))
                            {
// got complete frame
//                                 printf("FrameInput::reader_thread %d got frame %d bytes\n", 
//                                     __LINE__,
//                                     output_offset);

                                pthread_mutex_lock(&frame_lock);
                                memcpy(reader_buffer3, reader_buffer2, output_offset);
                                frame_size = output_offset;
                                pthread_mutex_unlock(&frame_lock);
                                sem_post(&frame_ready_sema);

                                frame_count++;
                                struct timeval time2;
                                gettimeofday(&time2, 0);
                                int64_t diff = TO_MS(time2) - TO_MS(time1);
                                if(diff >= 1000)
                                {
//                                     printf("FrameInput::reader_thread %d FPS: %f\n",
//                                         __LINE__,
//                                         (double)frame_count * 1000 / diff);
                                    frame_count = 0;
                                    gettimeofday(&time1, 0);
                                }

                                reader_state = GET_START_CODE;
                                code_offset = 0;
                            }
                        }
                        else
                        {
                            code_offset = 0;
                            if(c == stop_code[code_offset])
                            {
                                code_offset++;
                            }
                        }
                    }
                }
            }
        }
    }

    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> workProducer()
    {

        if(!initialized)
        {
// select the USB cam with our hack to cap_avfoundation_mac.mm
//            cap = cv::VideoCapture(1);

	        pthread_mutexattr_t attr;
	        pthread_mutexattr_init(&attr);
	        pthread_mutex_init(&frame_lock, &attr);
            sem_init(&frame_ready_sema, 0, 0);

	        pthread_t reader_tid;
	        pthread_create(&reader_tid, 
		        0, 
		        reader_thread, 
		        0);

            initialized = 1;
//printf("FrameInput::workProducer %d\n", __LINE__);
//sleep(1000000);
        }

// Create new datum
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();
        int frame_size2;

        while(1)
        {
            sem_wait(&frame_ready_sema);
            pthread_mutex_lock(&frame_lock);
            frame_size2 = frame_size;
            if(frame_size2 > 0)
            {
                cv::Mat rawData(1, frame_size, CV_8UC1, (void*)reader_buffer3);
                datumPtr->cvInputData = imdecode(rawData, cv::IMREAD_COLOR);
                frame_size = 0;
            }
            pthread_mutex_unlock(&frame_lock);
            if(frame_size2 > 0)
            {
                break;
            }
        }

        struct timeval time2;
        gettimeofday(&time2, 0);


//         printf("FrameInput %d size=%d w=%d h=%d\n", 
//             __LINE__, 
//             frame_size2,
//             datumPtr->cvInputData.cols, 
//             datumPtr->cvInputData.rows);

        return datumsPtr;
    }
};

int FrameInput::initialized = 0;
int FrameInput::frame_count = 0;
FILE* FrameInput::fd = 0;
int FrameInput::reader_state = GET_START_CODE;
int FrameInput::code_offset = 0;
int FrameInput::output_offset = 0;
const uint8_t FrameInput::start_code[4] = { 0xff, 0xd8, 0xff, 0xdb };
const uint8_t FrameInput::stop_code[2] = { 0xff, 0xd9 };
unsigned char FrameInput::reader_buffer1[BUFSIZE1];
unsigned char FrameInput::reader_buffer2[BUFSIZE2];
unsigned char FrameInput::reader_buffer3[BUFSIZE2];
int FrameInput::frame_size = 0;
pthread_mutex_t FrameInput::frame_lock;
// FrameInput waits for this
sem_t FrameInput::frame_ready_sema;

#endif // LOAD_GPHOTO2








void quit(int sig)
{
// reset the console
	struct termios info;
	tcgetattr(fileno(stdin), &info);
	info.c_lflag |= ICANON;
	info.c_lflag |= ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &info);
    exit(0);
}



// void* configure_thread(void *ptr)
// {
//     int result = init_servos();
//     if(result)
//     {
// // don't use the camera mount
//         processing = 1;
//         return 0;
//     }
// 
// 	struct termios term_info;
// 	struct termios default_term_info;
//     int in_fd = -1;
// 	in_fd = fileno(stdin);
// 	tcgetattr(in_fd, &default_term_info);
// 	tcgetattr(in_fd, &term_info);
// 	term_info.c_lflag &= ~ICANON;
// 	term_info.c_lflag &= ~ECHO;
// 	tcsetattr(in_fd, TCSANOW, &term_info);
// 
// 
// // printf("main %d pan=%d tilt=%d pan_sign=%d tilt_sign=%d\n", 
// // __LINE__,
// // (int)(pan - (MAX_PWM + MIN_PWM) / 2),
// // (int)(tilt - (MAX_PWM + MIN_PWM) / 2),
// // pan_sign,
// // tilt_sign);
// 
//     printf("-----------------------------------------------------\n");
//     printf("Welcome to the tracker\n");
//     printf("Press SPACE or ENTER to activate mount\n");
//     printf("ESC to give up & go to a movie.\n");
//     while(1)
//     {
//         uint8_t c;
//         int _ = read(in_fd, &c, 1);
// 
//         if(c == ' ' ||
//             c == '\n')
//         {
//             break;
//         }
// 
//         if(c == 27)
//         {
//             quit(0);
//             break;
//         }
//     }
// 
// // printf("main %d pan=%d tilt=%d pan_sign=%d tilt_sign=%d\n", 
// // __LINE__,
// // (int)(pan - (MAX_PWM + MIN_PWM) / 2),
// // (int)(tilt - (MAX_PWM + MIN_PWM) / 2),
// // pan_sign,
// // tilt_sign);
// 
// // write it a few times to defeat UART initialization glitches
//     write_servos(1);
//     usleep(100000);
//     write_servos(1);
//     usleep(100000);
//     write_servos(1);
//     usleep(100000);
//     write_servos(1);
// 
// // printf("main %d pan=%d tilt=%d pan_sign=%d tilt_sign=%d\n", 
// // __LINE__,
// // (int)(pan - (MAX_PWM + MIN_PWM) / 2),
// // (int)(tilt - (MAX_PWM + MIN_PWM) / 2),
// // pan_sign,
// // tilt_sign);
// 
//     printf("-----------------------------------------------------\n");
//     printf("Press keys to aim mount.\n");
//     printf("PWM Values should be as close to 0 as possible.\n");
//     printf("a - left\n");
//     printf("d - right\n");
//     printf("w - up\n");
//     printf("s - down\n");
//     printf("p - reverse pan sign\n");
//     printf("t - reverse tilt sign\n");
//     printf("SPACE or ENTER to save defaults & begin tracking\n");
//     printf("ESC to give up & go to a movie.\n");
//     int done = 0;
//     while(!done)
//     {
//         uint8_t c;
//         int print_servos = 0;
//         int _ = read(in_fd, &c, 1);
// 
//         switch(c)
//         {
//             case 27:
//                 save_defaults();
//                 quit(0);
//                 break;
// 
//             case ' ':
//             case '\n':
//                 printf("\nBeginning tracking\n");
//                 done = 1;
//                 break;
//             case 'a':
//                 pan -= PAN_STEP * pan_sign;
//                 write_servos(1);
//                 print_servos = 1;
//                 break;
//             case 'd':
//                 pan += PAN_STEP * pan_sign;
//                 write_servos(1);
//                 print_servos = 1;
//                 break;
//             case 's':
//                 tilt -= TILT_STEP * tilt_sign;
//                 write_servos(1);
//                 print_servos = 1;
//                 break;
//             case 'w':
//                 tilt += TILT_STEP * tilt_sign;
//                 write_servos(1);
//                 print_servos = 1;
//                 break;
//             case 't':
//                 tilt_sign *= -1;
//                 print_servos = 1;
//                 break;
//             case 'p':
//                 pan_sign *= -1;
//                 print_servos = 1;
//                 break;
//         }
// 
//         if(print_servos)
//         {
//             printf("main %d pan=%d tilt=%d pan_sign=%d tilt_sign=%d       \r", 
//                 __LINE__,
//                 (int)(pan - (MAX_PWM + MIN_PWM) / 2),
//                 (int)(tilt - (MAX_PWM + MIN_PWM) / 2),
//                 pan_sign,
//                 tilt_sign);
//             fflush(stdout);
//         }
//     }
// 
// 	tcsetattr(in_fd, TCSANOW, &default_term_info);
//     close(in_fd);
// 
//     start_pan = pan;
//     start_tilt = tilt;
//     save_defaults();
// 
//     processing = 1;
// }



class Process : public op::WorkerConsumer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
    struct timespec time1;

    void initializationOnThread() 
    {
        clock_gettime(CLOCK_MONOTONIC, &time1);
        
    }
   
    
    void reset_zone(zone_t *zone, int *parts)
    {
        zone->min_y = 0;
        zone->max_y = 0;
        zone->total = 0;
        zone->parts = parts;
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

    void workConsumer(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
    {
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
// get frame period in seconds
            struct timespec time2;
            clock_gettime(CLOCK_MONOTONIC, &time2);
            double delta = 
                (double)((time2.tv_sec * 1000 + time2.tv_nsec / 1000000) -
                (time1.tv_sec * 1000 + time1.tv_nsec / 1000000)) / 1000;
            time1 = time2;
            if(!have_time1)
            {
                delta = 0;
            }
            have_time1 = 1;


// printf("Process::workConsumer %d %d %d delta=%f\n", 
// __LINE__, 
// sizeof(time2.tv_sec), 
// sizeof(time2.tv_nsec), 
// delta);


            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            const auto& image = datumsPtr->at(0)->cvOutputData;

            if(current_operation == TRACKING)
            {

                int humans = poseKeypoints.getSize(0);
                if(humans > MAX_HUMANS)
                {
                    humans = MAX_HUMANS;
                }
//             printf("Process::workConsumer %d frame=%d humans=%d w=%d h=%d\n", 
//                 __LINE__, 
//                 frames,
//                 humans,
//                 width,
//                 height);


                for (int human = 0 ; 
                    human < humans; 
                    human++)
                {
                    const auto& bodyParts = poseKeypoints.getSize(1);
                    body_t *body = &bodies[human];
// reset the zones
                    for(int i = 0; i < TOTAL_ZONES; i++)
                    {
                        reset_zone(&body->zones[i], zone_parts[i]);
                    }


                    if(bodyParts >= 25)
                    {
// get bounding box of all body parts for now
                        int have_body_part = 0;
                        for(int body_part = 0; body_part < BODY_PARTS; body_part++)
                        {
                            int x = (int)poseKeypoints[{ human, body_part, 0 }];
                            int y = (int)poseKeypoints[{ human, body_part, 1 }];

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

// get the head size from the zones
                        if(body->zones[NECK_ZONE].total &&
                            body->zones[HEAD_ZONE].total)
                        {
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

// focal point of all humans
                float total_x = 0;
                float total_y = 0;
                int width = image.cols;
                int height = image.rows;
                int center_x = width / 2;
                int center_y = height / 2;
                int biggest_body = 0;
                int biggest_h = 0;
                zone_t zones[TOTAL_ZONES];
                int head_size = 0;

                for(int human = 0; human < humans; human++)
                {
                    body_t *body = &bodies[human];
// average center X of all bodies
                    total_x += (body->x1 + body->x2) / 2;

// use the tallest body for vertical measurements
                    if(body->y2 - body->y1 > biggest_h)
                    {
                        biggest_body = human;
                        biggest_h = body->y2 - body->y1;
                        total_y += (body->y1 + body->y2) / 2;
                        head_size = body->head_size;
                        for(int i = 0; i < TOTAL_ZONES; i++)
                        {
                            memcpy(&zones[i], &body->zones[i], sizeof(zone_t));
                        }
                    }
                }


// convert to EOS RP frame size
                head_size = NORMALIZE_DISTANCE(head_size);

                if(humans > 0)
                {
                    total_x /= humans;


                    int x_error = total_x - center_x;
    // printf("workConsumer %d: total_x=%d x1=%d x2=%d x_error=%d y_error=%d\n", 
    // __LINE__, (int)total_x, bodies[0].x1, bodies[0].x2, x_error, y_error);
                    float pan_change = delta * 
                        lenses[lens].x_gain * 
                        NORMALIZE_DISTANCE(x_error);
                    CLAMP(pan_change, 
                        -lenses[lens].max_pan_change, 
                        lenses[lens].max_pan_change);
                    pan += pan_change * pan_sign;


#ifdef TRACK_TILT
// top_y depends on the head size
                    int top_y = height / 8;
                    int top_y1 = top_y;
                    int top_y2 = height / 3;
// EOS RP frame sizes
                    int head_size1 = 150;
                    int head_size2 = 250;
                    if(head_size >= head_size1 && head_size < head_size2)
                    {
                        double fraction = (double)(head_size - head_size1) / 
                            (head_size2 - head_size1);
                        top_y = (int)(fraction * top_y2 + 
                            (1.0 - fraction) * top_y1);
                    }
                    else
                    if(head_size >= head_size2)
                    {
                        top_y = top_y2;
                    }
printf("Process: workConsumer %d: head_size=%d top_y=%d\n", 
__LINE__, 
head_size,
top_y);
                
                    int y_error = 0;
                    float tilt_change = 0;
// all zones visible.  Track the center or the head.
                    if(zones[0].total > 0 &&
                        zones[1].total > 0 &&
                        zones[2].total > 0 &&
                        zones[3].total > 0)
                    {
// limit the head
                        if(zones[0].min_y < top_y)
                        {
                            y_error = zones[0].min_y - top_y;
                        }
                        else
                        {
                            y_error = total_y - center_y;
                        }

                    }
                    else
    // head is visible but other zones are hidden.  Track the head.
                    if(zones[0].total > 0)
                    {
                        y_error = zones[0].min_y - top_y;
                    }
    // other zones are visible but head is hidden.  Tilt up.
                    else
                    if(zones[1].total > 0 ||
                        zones[2].total > 0 ||
                        zones[3].total > 0)
                    {
                        y_error = -lenses[lens].tilt_search;
                    }

                    if(y_error != 0)
                    {
                        tilt_change = delta * 
                            lenses[lens].y_gain * 
                            NORMALIZE_DISTANCE(y_error);
                        CLAMP(tilt_change, 
                            -lenses[lens].max_tilt_change, 
                            lenses[lens].max_tilt_change);
                        tilt -= tilt_change * tilt_sign;
                    }
    #endif // TRACK_TILT

//                printf("pan_change=%d tilt_change=%d\n", (int)pan_change, (int)tilt_change);
//                    printf("pan=%d tilt=%d\n", (int)(pan - start_pan), (int)(tilt - start_tilt));

                    write_servos(0);
                }
            } // current_operation == TRACKING

            frames++;
            
            
            
            // show the output frame on the GUI
            if(current_operation == CONFIGURING ||
                current_operation == STARTUP)
            {
                int dst_h = WINDOW_H * 3 / 4;
                int dst_w = dst_h * image.cols / image.rows;
                draw_video((unsigned char*)image.ptr(0), 
                    WINDOW_W - dst_w,
                    WINDOW_H - dst_h,
                    dst_w,
                    dst_h,
                    image.cols,
                    image.rows,
                    image.cols * 3);
            }
            else
            {
                int dst_h = WINDOW_H;
                int dst_w = WINDOW_H * image.cols / image.rows;
                draw_video((unsigned char*)image.ptr(0), 
                    WINDOW_W / 2 - dst_w / 2,
                    0,
                    dst_w, 
                    dst_h,
                    image.cols,
                    image.rows,
                    image.cols * 3);
            }
        }
        
        
        
         
    }
 

    
};




int main(int argc, char *argv[])
{
// set up the camera mount
    signal(SIGHUP, quit);
    signal(SIGINT, quit);
    signal(SIGQUIT, quit);
    signal(SIGTERM, quit);

    load_defaults();

    init_gui();
    int result = init_servos();
    if(result)
    {
// don't use the camera mount
        current_operation = TRACKING;
    }
    else
    {
        gui->lock_window();
        gui->set_font(LARGEFONT);
        gui->set_color(WHITE);
        int text_h = gui->get_text_height(LARGEFONT, "q0");
        int y = text_h + MARGIN;
        int x = MARGIN;
        gui->draw_text(x, 
            y, 
            "Welcome to the tracker\n\n"
            "Press SPACE or ENTER to activate the mount\n\n"
            "ESC to give up & go to a movie.");
        gui->flash(1);
        gui->unlock_window();
    }


// custom input
    op::Wrapper opWrapper;
#ifdef LOAD_GPHOTO2
    opWrapper.setWorker(op::WorkerType::Input, // workerType
        std::make_shared<FrameInput>(), // worker
        false); // const bool workerOnNewThread
#endif

#ifdef LOAD_WEBCAM
    op::ProducerType producerType;
    std::string producerString;
    std::tie(producerType, producerString) = op::flagsToProducer(
        "", // FLAGS_image_dir
        "", // FLAGS_video
        "", // FLAGS_ip_camera
//        -1, // FLAGS_camera // /dev/video0
        1, // FLAGS_camera // /dev/video1
        false, // FLAGS_flir_camera
        -1); // FLAGS_flir_camera_index

    const auto cameraSize = op::flagsToPoint(
        "-1x-1", // FLAGS_camera_resolution
        "-1x-1");
    const op::WrapperStructInput wrapperStructInput
    {
        producerType, 
        producerString, 
        0, // FLAGS_frame_first, 
        1, // FLAGS_frame_step, 
        (unsigned long long)-1, // FLAGS_frame_last,
        false, // FLAGS_process_real_time, 
        false, // FLAGS_frame_flip, 
        0, // FLAGS_frame_rotate, 
        false, // FLAGS_frames_repeat,
        cameraSize, 
        "", // FLAGS_camera_parameter_path, 
        false, // FLAGS_frame_undistort, 
        -1, // FLAGS_3d_views
    };
    opWrapper.configure(wrapperStructInput);
#endif // LOAD_WEBCAM


    opWrapper.setWorker(op::WorkerType::Output, // workerType
        std::make_shared<Process>(), // worker
        false); // const bool workerOnNewThread


// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
//    const auto netInputSize = op::flagsToPoint("-1x160", "-1x160");
    const auto netInputSize = op::flagsToPoint("-1x256", "-1x256");
//    const auto netInputSize = op::flagsToPoint("-1x368", "-1x368");
    const auto outputSize = op::flagsToPoint("-1x-1", "-1x-1");
    const auto keypointScale = op::flagsToScaleMode(0);
    const auto multipleView = false;
    const auto poseModel = op::flagsToPoseModel("BODY_25");
    const auto heatMapTypes = op::flagsToHeatMaps(false, false, false);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(2);
    const bool enableGoogleLogging = true;

    const op::WrapperStructPose wrapperStructPose
    {
        op::PoseMode::Enabled, // PoseMode
        netInputSize, // netInputSize
        outputSize, 
        keypointScale, 
        -1, // FLAGS_num_gpu
        0, // FLAGS_num_gpu_start
        1, // scale_number
        (float)0.25, // FLAGS_scale_gap
        op::flagsToRenderMode(-1, multipleView),
        poseModel, 
        true, // !FLAGS_disable_blending
        (float)0.6, // FLAGS_alpha_pose
        (float)0.7, // FLAGS_alpha_heatmap
        0, // FLAGS_part_to_show
        MODELS, // FLAGS_model_folder
        heatMapTypes, 
        heatMapScale, 
        false, // FLAGS_part_candidates
        (float)0.05, // FLAGS_render_threshold
        MAX_HUMANS, // FLAGS_number_people_max
        false, // FLAGS_maximize_positives
        -1, // FLAGS_fps_max
        "", // FLAGS_prototxt_path
        "", // FLAGS_caffemodel_path
        (float)0, // (float)FLAGS_upsampling_ratio
        enableGoogleLogging
    };
    opWrapper.configure(wrapperStructPose);


// Face configuration (use op::WrapperStructFace{} to disable it)
//    const auto faceNetInputSize = op::flagsToPoint("368x368", "368x368 (multiples of 16)");
        
//     const op::WrapperStructFace wrapperStructFace
//     {
//         false, // FLAGS_face
//         faceNetInputSize, 
//         op::flagsToRenderMode(-1, multipleView, -1),
//         (float)0.6, // FLAGS_face_alpha_pose
//         (float)0.7, // FLAGS_face_alpha_heatmap
//         (float)0.4, // FLAGS_face_render_threshold
//     };
    const op::WrapperStructFace wrapperStructFace{};
    opWrapper.configure(wrapperStructFace);


// Hand configuration (use op::WrapperStructHand{} to disable it)
//    const auto handNetInputSize = op::flagsToPoint("368x368", "368x368 (multiples of 16)");
    
//     const op::WrapperStructHand wrapperStructHand
//     {
//         false, // FLAGS_hand
//         handNetInputSize, 
//         1, // FLAGS_hand_scale_number
//         (float)0.4, // FLAGS_hand_scale_range 
//         false, // FLAGS_hand_tracking
//         op::flagsToRenderMode(-1, multipleView, -1), 
//         (float)0.6, // FLAGS_hand_alpha_pose
//         (float)0.7, // FLAGS_hand_alpha_heatmap
//         (float)0.2, // FLAGS_hand_render_threshold
//     };
    const op::WrapperStructHand wrapperStructHand{};
    opWrapper.configure(wrapperStructHand);

// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    const op::WrapperStructExtra wrapperStructExtra
    {
        false, // FLAGS_3d
        -1, // FLAGS_3d_min_views
        false, // FLAGS_identification
        -1, // FLAGS_tracking
        0 // FLAGS_ik_threads
    };
    opWrapper.configure(wrapperStructExtra);


#ifdef LOAD_TEST_INPUT
    op::ProducerType producerType;
    std::string producerString;
    std::tie(producerType, producerString) = op::flagsToProducer(
        "test_input", // FLAGS_image_dir
        "", // FLAGS_video
        "", // FLAGS_ip_camera
        -1, // FLAGS_camera -1: default webcam
        false, // FLAGS_flir_camera
        -1); // FLAGS_flir_camera_index
    const auto cameraSize = op::flagsToPoint("-1x-1", "-1x-1");



    const op::WrapperStructInput wrapperStructInput
    {
        producerType, 
        producerString, 
        0, // FLAGS_frame_first
        1, // FLAGS_frame_step
        (unsigned long long)-1, // FLAGS_frame_last
        false, // FLAGS_process_real_time
        false, // FLAGS_frame_flip
        0, // FLAGS_frame_rotate
        false, // FLAGS_frames_repeat
        cameraSize, 
        "models/cameraParameters/flir/", // FLAGS_camera_parameter_path
        false, // FLAGS_frame_undistort
        -1, // FLAGS_3d_views
    };
    opWrapper.configure(wrapperStructInput);
#endif // LOAD_TEST_INPUT



#if defined(SAVE_OUTPUT)
// write output frames to test_output
    const op::WrapperStructOutput wrapperStructOutput
    {
        -1.f, // FLAGS_cli_verbose
        "", // FLAGS_write_keypoint
        op::stringToDataFormat("yml"), // FLAGS_write_keypoint_format
        "", // FLAGS_write_json
        "", // FLAGS_write_coco_json
        1, // FLAGS_write_coco_json_variants
        0, // FLAGS_write_coco_json_variant
        OUTPATH, // FLAGS_write_images
        "jpg", // FLAGS_write_images_format
        "", // FLAGS_write_video
        -1., // FLAGS_write_video_fps
        false, // FLAGS_write_video_with_audio
        "", // FLAGS_write_heatmaps
        "png", // FLAGS_write_heatmaps_format
        "", // FLAGS_write_video_3d
        "", // FLAGS_write_video_adam
        "", // FLAGS_write_bvh
        "", // FLAGS_udp_host
        "", // FLAGS_udp_port
    };
    opWrapper.configure(wrapperStructOutput);


#endif // SAVE_OUTPUT


// GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui
    {
// show a window on the screen
//        op::flagsToDisplayMode(-1, false), 
// draw info on the exported frames
        op::flagsToDisplayMode(0, false), 
        true, // guiVerbose
        false, // FLAGS_fullscreen.  Doesn't work.
    };
    opWrapper.configure(wrapperStructGui);

    opWrapper.exec();

    quit(0);
}


















