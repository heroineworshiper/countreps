/*
 * tracking camera
 * Copyright (C) 2019 Adam Williams <broadcast at earthling dot net>
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

// this runs on a macbook to do the machine vision
// be sure to install the FTDI driver for Macos, 
// make sure the virtual machine isn't bound to it.
// the right webcam is being selected

// To build tracker:
// 
// LD_LIBRARY_PATH=lib/ make tracker
// 
// To run it, specify the library path:
// 
// LD_LIBRARY_PATH=lib/ ./tracker
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

// read frames from test_input instead of webcam
//#define LOAD_INPUT

// save openpose output in test_output
//#define SAVE_OUTPUT

#define MAX_HUMANS 2
// tilt moves down for lower numbers
// with DSLR
float pan = 24976;
float tilt = 16976;
// with webcam for testing
//float pan = 24976;
//float tilt = 25776;

// pan moves right for lower numbers
#define PAN_MIN 18676
#define PAN_MAX 29176
// with DSLR
#define TILT_MIN 14976
#define TILT_MAX 30676
// with webcam for testing
//#define TILT_MIN 24076
//#define TILT_MAX 28176

#ifdef __clang__
#define MODELS "../openpose.mac/models/"
#else
#define MODELS "../openpose/models/"
#endif

#define TEXTLEN 1024
#define BODY_PARTS 25

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define GAIN 0.5

typedef struct
{
    int x1, y1, x2, y2;
} rect_t;

rect_t rects[MAX_HUMANS];
int last_x = -1;
int last_y = -1;
int center_x;
int center_y;
int width;
int height;


int servo_fd = -1;
int frames = 0;

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


void init_servos()
{
#ifdef __clang__
    servo_fd = init_serial("/dev/cu.usbserial-AL03OO1F");
#else
	servo_fd = init_serial("/dev/ttyACM0");
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM1");
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM2");
#endif
}

void write_servos()
{
	if(servo_fd >= 0)
	{
#define SYNC_CODE 0xe5
#define BUFFER_SIZE 5
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


class Process : public op::WorkerConsumer<std::shared_ptr<std::vector<op::Datum>>>
{
public:
    static int servo_fd;

    void initializationOnThread() {}
   
    

    void workConsumer(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
    {
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
            const auto& image = datumsPtr->at(0).cvOutputData;
            width = image.cols;
            height = image.rows;
            center_x = width / 2;
            center_y = height * 1 / 4;    
//            center_y = height / 2;
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
                rect_t *rect = &rects[human];
                
                if(bodyParts >= 25)
                {
// get bounding box of all body parts for now
                    int have_body_part = 0;
                    for(int body_part = 0; body_part < BODY_PARTS; body_part++)
                    {
                        int x = (int)poseKeypoints[{ human, body_part, 0 }];
                        int y = (int)poseKeypoints[{ human, body_part, 1 }];

                        if(x > 0 && y > 0)
                        {
                            if(!have_body_part)
                            {
                                rect->x1 = rect->x2 = x;
                                rect->y1 = rect->y2 = y;
                                have_body_part = 1;
                            }
                            else
                            {
                                if(rect->x1 > x)
                                {
                                    rect->x1 = x;
                                }

                                if(rect->x2 < x)
                                {
                                    rect->x2 = x;
                                }

                                if(rect->y1 > y)
                                {
                                    rect->y1 = y;
                                }

                                if(rect->y2 < y)
                                {
                                    rect->y2 = y;
                                }
                            }
                        }
                    }
                }
            }
            
// focal point of all humans
            float total_x = 0;
            float total_y = 0;
            for(int human = 0; human < humans; human++)
            {
                rect_t *rect = &rects[human];
// average top center X
                total_x += (rect->x1 + rect->x2) / 2;
// take highest Y
                if(human == 0 || total_y > rect->y1)
                {
                    total_y = rect->y1;
                }
// take center Y
//                total_y += (rect->y1 + rect->y2) / 2;
            }
// want the total_ point to be here

            if(humans > 0)
            {
                total_x /= humans;
                last_x = total_x;
                last_y = total_y;
            

                int x_error = total_x - center_x;
                int y_error = total_y - center_y;
printf("workConsumer %d: total_x=%d x1=%d x2=%d x_error=%d y_error=%d\n", 
__LINE__, (int)total_x, rects[0].x1, rects[0].x2, x_error, y_error);
                float pan_change = GAIN * x_error;
                float tilt_change = GAIN * y_error;
                pan -= pan_change;
                tilt -= tilt_change;
                CLAMP(pan, PAN_MIN, PAN_MAX);
                CLAMP(tilt, TILT_MIN, TILT_MAX);
                write_servos();
            }
//             else
// // use last known coords
//             if(last_x > 0 && last_y > 0)
//             {
//                 int x_error = last_x - center_x;
//                 int y_error = last_y - center_y;
//                 float pan_change = GAIN * x_error;
//                 float tilt_change = GAIN * y_error;
//                 pan -= pan_change;
//                 CLAMP(pan, PAN_MIN, PAN_MAX);
//                 CLAMP(tilt, TILT_MIN, TILT_MAX);
//                 write_servos();
//             }

            frames++;
        }
        
        
        
         
    }
 

    
};

int main(int argc, char *argv[])
{
    init_servos();
    write_servos();

// start the pose estimator
    op::Wrapper opWrapper;
    opWrapper.setWorker(op::WorkerType::Output, // workerType
        std::make_shared<Process>(), // worker
        false); // const bool workerOnNewThread


// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const auto netInputSize = op::flagsToPoint("-1x160", "-1x160");
//    const auto netInputSize = op::flagsToPoint("-1x256", "-1x256");
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
        true, // !FLAGS_body_disable
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
        1, // FLAGS_number_people_max
        false, // FLAGS_maximize_positives
        -1, // FLAGS_fps_max
        enableGoogleLogging
    };
    opWrapper.configure(wrapperStructPose);


// Face configuration (use op::WrapperStructFace{} to disable it)
    const auto faceNetInputSize = op::flagsToPoint("368x368", "368x368 (multiples of 16)");
        
    const op::WrapperStructFace wrapperStructFace
    {
        false, // FLAGS_face
        faceNetInputSize, 
        op::flagsToRenderMode(-1, multipleView, -1),
        (float)0.6, // FLAGS_face_alpha_pose
        (float)0.7, // FLAGS_face_alpha_heatmap
        (float)0.4, // FLAGS_face_render_threshold
    };
    opWrapper.configure(wrapperStructFace);


// Hand configuration (use op::WrapperStructHand{} to disable it)
    const auto handNetInputSize = op::flagsToPoint("368x368", "368x368 (multiples of 16)");
    
    const op::WrapperStructHand wrapperStructHand
    {
        false, // FLAGS_hand
        handNetInputSize, 
        1, // FLAGS_hand_scale_number
        (float)0.4, // FLAGS_hand_scale_range 
        false, // FLAGS_hand_tracking
        op::flagsToRenderMode(-1, multipleView, -1), 
        (float)0.6, // FLAGS_hand_alpha_pose
        (float)0.7, // FLAGS_hand_alpha_heatmap
        (float)0.2, // FLAGS_hand_render_threshold
    };
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


// Get frames from test_input
    op::ProducerType producerType;
    std::string producerString;
    std::tie(producerType, producerString) = op::flagsToProducer(


#ifdef LOAD_INPUT
        INPATH, // FLAGS_image_dir
#else
        "", // FLAGS_image_dir
#endif
        "", // FLAGS_video
        "", // FLAGS_ip_camera
        1, // FLAGS_camera -1: default webcam
        false, // FLAGS_flir_camera
        -1); // FLAGS_flir_camera_index
    const auto cameraSize = op::flagsToPoint("-1x-1", "-1x-1");
//    const auto cameraSize = op::flagsToPoint("640x360", "640x360");



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
    



#if defined(SAVE_OUTPUT)
// write output frames to test_output
    const op::WrapperStructOutput wrapperStructOutput
    {
        -1.f, // FLAGS_cli_verbose
        "", // FLAGS_write_keypoint
        op::stringToDataFormat("yml"),
        "", // FLAGS_write_json
        "", // FLAGS_write_coco_json
        "", // FLAGS_write_coco_foot_json
        0, // FLAGS_write_coco_json_variant
        OUTPATH, // FLAGS_write_images
        "jpg", // FLAGS_write_images_format
        "", // FLAGS_write_video
        -1., // FLAGS_write_video_fps
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
        op::flagsToDisplayMode(-1, false), 
        true, 
        true, // FLAGS_fullscreen
    };
    opWrapper.configure(wrapperStructGui);


    opWrapper.exec();

}


















