/*
 * countreps
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








// To build it:
// 
// ./make.sh
// 
// To run it:
// 
// ./countreps.sh
// 
// install kernel module on Linux:
// insmod /lib/modules/4.9.39/kernel/drivers/video/nvidia-uvm.ko


// OpenPose dependencies
#include <openpose/headers.hpp>

// data ingestor
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>


#ifdef __clang__
#include <dispatch/dispatch.h>
#else
#include <semaphore.h>
#endif


// calculate the coordinates from test_input & write them to DEBUG_COORDS
// write output frames in test_output
//#define DO_TEST_PHOTOS

    // test photos go here
    #define INPATH "test_input"
    #define OUTPATH "test_output"
    // coordinates go here
    #define DEBUG_COORDS "debug_coords"

// read the coordinates from DEBUG_COORDS & print the results without
// using images
//#define LOAD_COORDS


// start a server & wait for connections from the tablet
#define DO_SERVER
    #define PORT1 1234
    #define PORT2 1244
    #define BUFFER 0x100000
    // read frames from test_input instead of the tablet
    //#define SERVER_READFILES
    // save frames from the tablet in test_input
    #define SAVE_INPUT
    // save openpose output in test_output
    #define SAVE_OUTPUT
    // save coordinates in DEBUG_COORDS
    #define SAVE_COORDS

// show a window on the screen
//#define USE_GUI



#ifdef __clang__
#define MODELS "../openpose.mac/models/"
#else
#define MODELS "../openpose/models/"
#endif

#define TEXTLEN 1024

// reps must be separated by this many frames
#define DEBOUNCE 4
// minimum ms between frames
#define MINIMUM_PERIOD 150

// the body parts as defined in 
// src/openpose/pose/poseParameters.cpp: POSE_BODY_25_BODY_PARTS
#define MODEL_NECK 1
#define MODEL_BUTT 8
#define MODEL_HIP1 12
#define MODEL_HIP2 9
#define MODEL_KNEE1 13
#define MODEL_KNEE2 10
#define MODEL_ELBOW1 6
#define MODEL_ELBOW2 3
#define MODEL_SHOULDER1 5
#define MODEL_SHOULDER2 2
#define MODEL_ANKLE1 14
#define MODEL_ANKLE2 11
#define MODEL_WRIST2 4
#define BODY_PARTS 25


// exercises
#define UNKNOWN_EXERCISE -1
#define SITUPS 0
#define PUSHUPS 1
#define HIP_FLEX 2
#define SQUAT 3
#define TOTAL_EXERCISES 4


// exercise poses
#define UNKNOWN_POSE -1
#define SITUP_DOWN 0
#define SITUP_UP 1
#define PUSHUP_DOWN 2
#define PUSHUP_UP 3
#define HIP_UP 4
#define STANDING 5
#define SQUAT_DOWN 6



#define TORAD(x) ((float)(x) * 2 * M_PI / 360)
#define TODEG(x) ((float)(x) * 360 / M_PI / 2)

typedef struct
{
    float x;
    float y;
} coord_t;


typedef struct
{
    int exercise;
    int reps;
} exercise_plan_t;


// the exercise plan
exercise_plan_t plan[] = 
{
    { SITUPS, 30 },
    { PUSHUPS, 30 },
    { HIP_FLEX, 30 },
    { HIP_FLEX, 30 },
    
    { SITUPS, 30 },
    { PUSHUPS, 30 },
    { SQUAT, 30 },
    
    { SITUPS, 30 },
    { PUSHUPS, 30 }
};
#define TOTAL_PLANS (sizeof(plan) / sizeof(exercise_plan_t))

// the current frame number
int frames = 0;





// move frames from the network to openpose
class FrameInput;

std::shared_ptr<FrameInput> frame_input = nullptr;
int server_socket = -1;

class FrameInput : public op::WorkerProducer<std::shared_ptr<std::vector<op::Datum>>>
{
public:
    int done = 0;
#ifndef __clang__
    sem_t frame_sema;
#else
    dispatch_semaphore_t frame_sema;
#endif

    pthread_mutex_t frame_lock;

    int have_frame = 0;
    uint8_t buffer[BUFFER];
    int frame_size;
    
    FrameInput()
    {
#ifndef __clang__
        sem_init(&frame_sema, 0, 0);
#else
        frame_sema = dispatch_semaphore_create(0);
#endif

        printf("FrameInput::FrameInput %d\n", __LINE__);
	    pthread_mutexattr_t attr;
	    pthread_mutexattr_init(&attr);
	    pthread_mutex_init(&frame_lock, &attr);
    }
    
    ~FrameInput()
    {
        printf("FrameInput::~FrameInput %d\n", __LINE__);

#ifndef __clang__
        sem_destroy(&frame_sema);
#else
        dispatch_release(frame_sema);
#endif

        pthread_mutex_destroy(&frame_lock);
    }

    void initializationOnThread() {}

    std::shared_ptr<std::vector<op::Datum>> workProducer()
    {
// wait for the next frame or quit
        while(1)
        {

#ifndef __clang__
            sem_wait(&frame_sema);
#else
            dispatch_semaphore_wait(frame_sema, DISPATCH_TIME_FOREVER);
#endif

//            printf("FrameInput::workProducer %d done=%d have_frame=%d\n", 
//                __LINE__, done, have_frame);

            pthread_mutex_lock(&frame_lock);
            if(done)
            {
                pthread_mutex_unlock(&frame_lock);
                this->stop();
                return nullptr;
            }
            else
            if(have_frame)
            {
                have_frame = 0;
                
                // Create new datum
                auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
                datumsPtr->emplace_back();
                auto& datum = datumsPtr->at(0);

#if defined(SAVE_INPUT) && !defined(SERVER_READFILES)
                char string[TEXTLEN];
                sprintf(string, "%s/frame%06d.jpg", 
                    INPATH,
                    frames);
                FILE *fd = fopen(string, "w");
                if(!fd)
                {
                    printf("FrameInput::workProducer %d couldn't open %s for writing.\n",
                        __LINE__,
                        string);
                }
                else
                {
                    fwrite(buffer, 1, frame_size, fd);
                    fclose(fd);
                }
                
#endif // SAVE_INPUT


                // Fill datum
                cv::Mat rawData(1, frame_size, CV_8UC1, (void*)buffer);
                datum.cvInputData = imdecode(rawData, cv::IMREAD_COLOR);

                pthread_mutex_unlock(&frame_lock);
                return datumsPtr;
            }
            else
            {
                pthread_mutex_unlock(&frame_lock);
            }
        }
    
        
    }
};







class Process : public op::WorkerConsumer<std::shared_ptr<std::vector<op::Datum>>>
{
public:
// the reps detected
    int reps = 0;
    int prev_reps = 0;
// frame the previous rep was detected on, for debouncing
    int prev_rep_frame = 0;
    int pose = UNKNOWN_POSE;
    int prev_pose = UNKNOWN_POSE;
// the current exercise
    int plan_line = 0;
    int exercise = plan[plan_line].exercise;
    uint8_t packet[BUFFER];

    void initializationOnThread() {}

// get raw data from openpose
    void workConsumer(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
    {
        if (datumsPtr != nullptr && !datumsPtr->empty())
        { 
            
            
            const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
            const auto& image = datumsPtr->at(0).cvOutputData;
            
            printf("Process::workConsumer %d frame=%d lions=%d\n", 
                __LINE__, 
                frames,
                poseKeypoints.getSize(0));
            for (auto lion = 0 ; lion < poseKeypoints.getSize(0) ; lion++)
            {
                const auto& bodyParts = poseKeypoints.getSize(1);
//                 printf("Process::workConsumer %d bodyparts=%d\n", 
//                     __LINE__, 
//                     bodyParts);


                if(bodyParts >= 25)
                {
// transfer the body parts to a simple array
                    coord_t output[BODY_PARTS];
                    for(int body_part = 0; body_part < BODY_PARTS; body_part++)
                    {
                        float x = poseKeypoints[{ lion, body_part, 0 }];
                        float y = poseKeypoints[{ lion, body_part, 1 }];
                        output[body_part].x = x;
                        output[body_part].y = y;
//                        printf("Process::workConsumer %d %f,%f\n", __LINE__, x, y);
                    }

                    process(output);
                }


            }



#ifdef DO_SERVER
            if(server_socket < 0)
            {
                this->stop();
            }
            else
            {

// send the image to the server
                std::vector<uchar> buff;
                std::vector<int> param(2);
                param[0] = cv::IMWRITE_JPEG_QUALITY;
                param[1] = 90;
                imencode(".jpg", image, buff, param);
                
                
                packet[0] = 0x9a;
                packet[1] = 0x54;
                packet[2] = 0x63;
                packet[3] = 0xd3;
                int image_size = buff.size();
                packet[4] = image_size & 0xff;
                packet[5] = (image_size >> 8) & 0xff;
                packet[6] = (image_size >> 16) & 0xff;
                packet[7] = (image_size >> 24) & 0xff;
                for(int i = 0; i < image_size; i++)
                {
                    packet[8 + i] = buff.at(i);
                }
                int _ignore = write(server_socket, packet, 8 + image_size);

// send the status to the server
                packet[0] = 0x9a;
                packet[1] = 0x54;
                packet[2] = 0x63;
                packet[3] = 0xd2;

// show the previous exercise reps if on the 1st rep of the next exercise
                int reps2 = reps;
                if(reps == 0 && plan_line > 0)
                {
                    reps2 = plan[plan_line - 1].reps;
                }
                packet[4] = reps2;
                packet[5] = exercise;
                _ignore = write(server_socket, packet, 6);
                
                
            }
#endif // DO_SERVER
            
            frames++;
        }
    }


// write coordinates to a file
    FILE *debug_fd = 0;
    void save_coords(coord_t *coords)
    {
        if(!debug_fd)
        {
            debug_fd = fopen(DEBUG_COORDS, "w");
        }
        
        char string[TEXTLEN];
        char string2[TEXTLEN];
        string2[0] = 0;
        sprintf(string2, "frame %d: ", frames);
        for(int i = 0; i < BODY_PARTS; i++)
        {
            sprintf(string, "%f,%f ", coords[i].x, coords[i].y);
            strcat(string2, string);
        }
        fprintf(debug_fd, "%s\n", string2);
        fflush(debug_fd);
    }





// convert the coordinates into reps
    void process(coord_t *coords)
    {


#if defined(SAVE_COORDS) && !defined(LOAD_COORDS)
        save_coords(coords);
#endif



//         char string[TEXTLEN];
//         char string2[TEXTLEN];
//         string2[0] = 0;
//         sprintf(string2, "frame %d: ", frames);
//         for(int i = 0; i < BODY_PARTS; i++)
//         {
//             sprintf(string, "%f,%f  ", coords[i * 2], coords[i * 2 + 1]);
//             strcat(string2, string);
//         }
//         printf("%s\n", string2);
        
        int failed = 0;

// classify the pose
        pose = get_exercise_pose(coords);



        calculate_reps();

        
        if(reps != prev_reps)
        {
            printf("Process.process2 %d: exercise=%d reps=%d\n", 
                __LINE__,
                exercise, 
                reps);
            prev_reps = reps;

// reset the counter
            if(plan_line < TOTAL_PLANS - 1)
            {
                if(reps >= plan[plan_line].reps)
                {
                    reps = 0;
                    prev_reps = 0;
                    prev_pose = UNKNOWN_POSE;
                    plan_line++;
                    exercise = plan[plan_line].exercise;
                    printf("Process.process2 %d: next exercise %d\n", __LINE__, exercise);
                }
            }
        }
    }



// returns if the body part was detected
    int have_coord(coord_t *coords, int body_part)
    {
        if(coords[body_part].x > 0.001 &&
            coords[body_part].y > 0.001)
        {
            return 1;
        }
        return 0;
    }

// get Y distance between 2 body parts
    float get_ydistance(coord_t *coords, 
        int body_part1, 
        int body_part2,
        int *have_it)
    {
        *have_it = 0;
        if(!have_coord(coords, body_part1) ||
            !have_coord(coords, body_part2))
        {
            return 0;
        }

        *have_it = 1;        
        return abs(coords[body_part2].y - coords[body_part1].y);
    }

// get the maximum Y distance of both legs
    float get_leg_dy(coord_t *coords, int *have_it)
    {
        *have_it = 0;

        float result = 0;
        int have = 0;
        float distance = get_ydistance(coords, MODEL_HIP1, MODEL_ANKLE1, &have);
        if(have)
        {
            result = distance;
            *have_it = 1;
        }


        distance = get_ydistance(coords, MODEL_HIP2, MODEL_ANKLE2, &have);
        if(have)
        {
            if(!*have_it || distance > result)
            {
                result = distance;
            }
            *have_it = 1;
        }

        if(!*have_it)
        {
            result = NAN;
        }

        return result;
    }

// get distance between 2 body parts
    float get_distance(coord_t *coords, 
        int body_part1, 
        int body_part2,
        int *have_it)
    {
        *have_it = 0;
        if(!have_coord(coords, body_part1) ||
            !have_coord(coords, body_part2))
        {
            return 0;
        }

        *have_it = 1;        
        return hypot(coords[body_part2].y - coords[body_part1].y,
            coords[body_part2].x - coords[body_part1].x);
    }

// get angle between 2 body parts
// right=0 up=90 down=-90 left=180/-180
    float get_angle(coord_t *coords, 
        int body_part1, 
        int body_part2,
        int *have_it)
    {
        *have_it = 0;
        if(!have_coord(coords, body_part1) ||
            !have_coord(coords, body_part2))
        {
            return 0;
        }

// body parts are too close together
        int have;
        float distance = get_distance(coords, 
            body_part1, 
            body_part2,
            &have);
        if(!have || distance < 10)
        {
            return 0;
        }

        *have_it = 1;        
        return -atan2(coords[body_part2].y - coords[body_part1].y,
            coords[body_part2].x - coords[body_part1].x);
    }

// get the maximum knee angle
    float get_knee_angle(coord_t *coords, int *have_it)
    {
        *have_it = 0;

        float result = 0;
        int have_knee = 0;
        float knee = get_angle(coords, MODEL_HIP1, MODEL_KNEE1, &have_knee);
        if(have_knee)
        {
            result = knee;
            *have_it = 1;
        }
        
        
        knee = get_angle(coords, MODEL_HIP2, MODEL_KNEE2, &have_knee);
        if(have_knee)
        {
            if(!*have_it || knee > result)
            {
                result = knee;
            }
            *have_it = 1;
        }


        if(!*have_it)
        {
            result = NAN;
        }
        return result;
    }




    int get_exercise_pose(coord_t *coords)
    {
        int have_butt = have_coord(coords, MODEL_BUTT);
        int have_neck = have_coord(coords, MODEL_NECK);
        int have_ankle1 = have_coord(coords, MODEL_ANKLE1);
        int have_ankle2 = have_coord(coords, MODEL_ANKLE2);
// Y distances
        int have_leg_dy = 0;
        float leg_dy = get_leg_dy(coords, &have_leg_dy);

// Y distance between ankles
        int have_ankle_dy;
        float ankle_dy = get_ydistance(coords, 
            MODEL_ANKLE1, 
            MODEL_ANKLE2,
            &have_ankle_dy);

        
        int have_back_dy = 0;
        float back_dy = get_ydistance(coords, 
            MODEL_NECK, 
            MODEL_BUTT,
            &have_back_dy);

// angles
        int have_back;
        float back = get_angle(coords, MODEL_BUTT, MODEL_NECK, &have_back);
        int have_knee;
        float knee = get_knee_angle(coords, &have_knee);
// assume always facing right
        int have_shoulder2;
        float shoulder2 = get_angle(coords, MODEL_SHOULDER2, MODEL_ELBOW2, &have_shoulder2);
        
        int have_elbow2;
        float elbow2 = get_angle(coords, MODEL_ELBOW2, MODEL_WRIST2, &have_elbow2);

        
        
        int result = UNKNOWN_POSE;


        switch(exercise)
        {
            case SITUPS:
                if(have_back && back > TORAD(135) &&
                    have_knee && knee > TORAD(25))
                {
                    result = SITUP_DOWN;
                }
                else
                if(have_back && back < TORAD(135) &&
                    have_back && back > TORAD(90) &&
                    have_knee && knee > TORAD(25))
                {
                    result = SITUP_UP;
                }
                break;
            
            case PUSHUPS:
// only test pushups facing right, because of occluded left elbow
                if(have_back && back > TORAD(0) &&
                    have_back && back < TORAD(45) &&
                    ((have_knee && knee > TORAD(135)) ||
                    (have_knee && knee < TORAD(-90))))
                {
                    if(have_elbow2 && 
                        have_shoulder2 &&
                        shoulder2 < TORAD(-45) &&
                        shoulder2 > TORAD(-120) &&
                        elbow2 < TORAD(-45) &&
                        elbow2 > TORAD(-120))
                    {
                        result = PUSHUP_UP;
                    }
                    else
                    if(have_elbow2 && 
                        have_shoulder2 &&
                        shoulder2 > TORAD(135) &&
                        elbow2 > TORAD(-90) &&
                        elbow2 < TORAD(0))
                    {
                        result = PUSHUP_DOWN;
                    }
                }

printf("Process.get_exercise_pose %d: frame=%d back=%d knee=%d shoulder=%d elbow=%d pose=%d\n",
__LINE__,
frames,
(int)back,
(int)knee,
(int)TODEG(shoulder2),
(int)TODEG(elbow2),
result);

                break;
            
            case HIP_FLEX:
// hip flex or squat flex
                if(have_back && back > TORAD(0) &&
                    have_back && back < TORAD(120) &&
                    have_knee && knee < TORAD(-60) &&
                    have_knee && knee > TORAD(-120))
                {
                    result = STANDING;
                }
// hip flex.  Can't differentiate side.
                else
                if(have_back && back > TORAD(60) && // back upright
                    have_back && back < TORAD(120) &&
                    have_knee && knee > TORAD(-30) && // knee raised
                    have_knee && knee < TORAD(30) && // not a squat
                    have_ankle_dy &&   // ankles spread
                    have_leg_dy &&
                    have_back_dy &&
                    ankle_dy > leg_dy / 3 &&
                    leg_dy > back_dy)
                {
                    result = HIP_UP;
                }
printf("Process.get_exercise_pose %d: frame=%d ankle_dy=%d leg_dy=%d back_dy=%d back=%d knee=%d pose=%d\n",
__LINE__,
frames,
(int)ankle_dy,
(int)leg_dy,
(int)back_dy,
(int)TODEG(back),
(int)TODEG(knee),
result);
                break;
            
            case SQUAT:
// hip flex or squat flex
                if(have_back && back > TORAD(0) &&
                    have_back && back < TORAD(120) &&
                    have_knee && knee < TORAD(-25) &&
                    have_knee && knee > TORAD(-120))
                {
                    result = STANDING;
                }
                else
// squat flex
                if(have_back && back > TORAD(0) &&
                    have_back && back < TORAD(90) &&
                    have_knee && knee > TORAD(25) &&
                    have_knee && knee < TORAD(90))
                {
                    result = SQUAT_DOWN;
                }
                break;
        }
        
//         printf("Process.get_exercise_pose %d: frame=%d ankle_dy=%d leg_dy=%d back_dy=%d back=%d knee=%d elbow=%d pose=%d\n",
//             __LINE__,
//             frames,
//             (int)ankle_dy,
//             (int)leg_dy,
//             (int)back_dy,
//             (int)TODEG(back),
//             (int)TODEG(knee),
//             (int)TODEG(elbow),
//             result);

        return result;
        

    }


// increment the reps counter with debouncing
    int increment_reps()
    {
        if(frames - prev_rep_frame >= DEBOUNCE)
        {
            reps++;
// only reset the delay if it counts, so a legitimate rep doesn't get dropped
//            prev_rep_frame = frames;
        }

// always reset the delay so a series of rapid oscillations doesn't get through
        prev_rep_frame = frames;
        
        
        return reps;
    }

    int calculate_reps()
    {
        switch(exercise)
        {
            case SITUPS:
                if(prev_pose == SITUP_DOWN && pose == SITUP_UP)
                {
                    increment_reps();
                    prev_pose = pose;
                }
                else
                if(pose == SITUP_DOWN || pose == SITUP_UP)
                {
                    prev_pose = pose;
                }
                break;

            case PUSHUPS:
                if(prev_pose == PUSHUP_UP && pose == PUSHUP_DOWN)
                {
                    increment_reps();
                    prev_pose = pose;
                }
                else
                if(pose == PUSHUP_UP || pose == PUSHUP_DOWN)
                {
                    prev_pose = pose;
                }
                break;


            case HIP_FLEX:
                if(prev_pose == STANDING && pose == HIP_UP)
                {
                    increment_reps();
                    prev_pose = pose;
                }
                else
                if(pose == STANDING || pose == HIP_UP)
                {
                    prev_pose = pose;
                }
                break;
            case SQUAT:
                if(prev_pose == STANDING && pose == SQUAT_DOWN)
                {
                    increment_reps();
                    prev_pose = pose;
                }
                else
                if(pose == STANDING || pose == SQUAT_DOWN)
                {
                    prev_pose = pose;
                }
                break;
        }
        
        return reps;
    }




};



void do_poser()
{


// start the pose estimator
    op::Wrapper opWrapper;
    opWrapper.setWorker(op::WorkerType::Output, // workerType
        std::make_shared<Process>(), // worker
        false); // const bool workerOnNewThread

// get input from a socket instead of test files
#ifdef DO_SERVER
    #ifndef SERVER_READFILES
    opWrapper.setWorker(op::WorkerType::Input, // workerType
        frame_input, // worker
        false); // const bool workerOnNewThread
    #endif // !SERVER_READFILES
#endif // DO_SERVER

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
#if !defined(DO_SERVER) || defined(SERVER_READFILES)
    op::ProducerType producerType;
    std::string producerString;
    std::tie(producerType, producerString) = op::flagsToProducer(
        INPATH, // FLAGS_image_dir
        "", // FLAGS_video
        "", // FLAGS_ip_camera
        -1, // FLAGS_camera
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
    
#endif //!DO_SERVER || SERVER_READFILES



#if defined(SAVE_OUTPUT) || defined(DO_TEST_PHOTOS)
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


#endif // SAVE_OUTPUT || DO_TEST_PHOTOS


// GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui
    {
#ifdef USE_GUI
// show a window on the screen
        op::flagsToDisplayMode(-1, false), 
#else // USE_GUI
// draw info on the exported frames
        op::flagsToDisplayMode(0, false), 
#endif // !USE_GUI
        true, 
        false, // FLAGS_fullscreen
    };
    opWrapper.configure(wrapperStructGui);


    opWrapper.exec();
}



void do_debug_file()
{
// read a debug file instead of using openpose to generate coordinates
    Process processor;
    FILE *in = fopen(DEBUG_COORDS, "r");
    if(!in)
    {
        printf("Couldn't open %s\n", DEBUG_COORDS);
        exit(1);
    }

    char string[TEXTLEN];
    while(!feof(in))
    {
        char *ptr = fgets(string, TEXTLEN, in);
        if(!ptr)
        {
            break;
        }

// skip 1 space
        while(*ptr != 0 && *ptr != ' ')
        {
            ptr++;
        }
        ptr++;

// extract frame number
        frames = atoi(ptr);

// skip 1 space
        while(*ptr != 0 && *ptr != ' ')
        {
            ptr++;
        }
        ptr++;

// extract floats
        coord_t coords[BODY_PARTS];
        for(int i = 0; i < BODY_PARTS; i++)
        {
            for(int j = 0; j < 2; j++)
            {
                float value;
                sscanf(ptr, "%f", &value);
                if(j == 0)
                {
                    coords[i].x = value;
                }
                else
                {
                    coords[i].y = value;
                }

// get next whitespace
                while(*ptr != 0 && *ptr != ' ' && *ptr != ',')
                {
                    ptr++;
                }

// skip next whitespace
                while(*ptr != 0 && (*ptr == ' ' || *ptr == ','))
                {
                    ptr++;
                }
            }
            
        }

        processor.process(coords);
    }
    
    exit(0);
}


unsigned char reader_frame[BUFFER];
unsigned char reader_buffer[BUFFER];
struct timeval last_frame_time;

// read frames from the socket on a thread
void* reader_thread(void *ptr)
{
    #define SERVER_CODE0 0
    #define SERVER_CODE1 1
    #define SERVER_CODE2 2
    #define SERVER_CODE3 3
    #define SERVER_SIZE  4
    #define SERVER_READ_FRAME 5
    int state = SERVER_CODE0;
    uint32_t frame_size = 0;
    int counter = 0;
    int bytes_read;
    gettimeofday(&last_frame_time, 0);
    
    
    while(1)
    {
        bytes_read = read(server_socket, reader_buffer, BUFFER);
        if(bytes_read <= 0)
        {
            break;
        }

        for(int i = 0; i < bytes_read; i++)
        {
            uint8_t c = reader_buffer[i];
            switch(state)
            {
                case SERVER_CODE0:
                    if(c == 0xb2)
                    {
                        state++;
                    }
                    break;
                case SERVER_CODE1:
                    if(c == 0x05)
                    {
                        state++;
                    }
                    else
                    {
                        state = SERVER_CODE0;
                    }
                    break;
                case SERVER_CODE2:
                    if(c == 0xad)
                    {
                        state++;
                    }
                    else
                    {
                        state = SERVER_CODE0;
                    }
                    break;
                case SERVER_CODE3:
                    if(c == 0x99)
                    {
                        state++;
                        counter = 0;
                        frame_size = 0;
                    }
                    else
                    {
                        state = SERVER_CODE0;
                    }
                    break;

                case SERVER_SIZE:
                    frame_size >>= 8;
                    frame_size |= ((uint32_t)c << 24);
                    counter++;
                    if(counter >= 4)
                    {
                        state++;
                        counter = 0;
                    }
                    break;

                case SERVER_READ_FRAME:
                    reader_frame[counter++] = c;
                    if(counter >= frame_size)
                    {
                        state = SERVER_CODE0;
//                         printf("reader_thread %d: frame_size=%d\n", 
//                             __LINE__, 
//                             frame_size);


// drop if it came too soon
                        struct timeval current_time;
                        gettimeofday(&current_time, 0);
                        int64_t time_diff = 
                            (current_time.tv_sec * 1000 + current_time.tv_usec / 1000) -
				            (last_frame_time.tv_sec * 1000 + last_frame_time.tv_usec / 1000);
                        if(time_diff >= MINIMUM_PERIOD)
                        {
                            last_frame_time =current_time;

// send to the poser
                            pthread_mutex_lock(&frame_input->frame_lock);
                            frame_input->have_frame = 1;
                            frame_input->frame_size = frame_size;
                            memcpy(frame_input->buffer, reader_frame, frame_size);
                            pthread_mutex_unlock(&frame_input->frame_lock);


#ifndef __clang__
                            sem_post(&frame_input->frame_sema);
#else
                            dispatch_semaphore_signal(frame_input->frame_sema);
#endif
                        }

                    }
                    break;
            }
        }

    }

    printf("reader_thread %d: connection closed\n", __LINE__);
    close(server_socket);
    server_socket = -1;

// signal the poser to finish
    frame_input->done = 1;

#ifndef __clang__
    sem_post(&frame_input->frame_sema);
#else
    dispatch_semaphore_signal(frame_input->frame_sema);
#endif
    
    return nullptr;
}


void do_server_connection()
{
// reset the counter
    frames = 0;
// create the frame source
    frame_input = std::make_shared<FrameInput>();
    
// create the reader thread
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_t reader_tid;

	pthread_create(&reader_tid, 
		0, 
		reader_thread, 
		0);


// openpose must run on the mane thread for mac
    
    do_poser();







    printf("do_server_connection %d: poser finished\n", __LINE__);

// wait for the poser to finish
    pthread_join(reader_tid, nullptr);
    printf("do_server_connection %d: reader finished\n", __LINE__);
}



void do_server()
{
    int socket_fd;
	struct sockaddr_in addr;
	if((socket_fd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("do_server %d: socket failed\n", __LINE__);
		return;
	}

    int got_it = 0;
    int port;
    for(port = PORT1; port < PORT2; port++)
    {
	    addr.sin_family = AF_INET;
	    addr.sin_port = htons((unsigned short)port);
	    addr.sin_addr.s_addr = htonl(INADDR_ANY);

	    if(bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) >= 0)
	    {
            got_it = 1;
            break;
	    }
    }
    
    if(!got_it)
    {
		printf("do_server %d: couldn't bind any port\n", __LINE__);
		return;
    }
    
	while(1)
	{
        printf("do_server %d: Waiting for connection on port %d\n", __LINE__, port);
		if(listen(socket_fd, 1) < 0)
		{
			printf("do_server %d: listen failed\n", __LINE__);
			return;
		}

		struct sockaddr_in clientname;
		socklen_t size = sizeof(clientname);
		if((server_socket = accept(socket_fd,
			(struct sockaddr*)&clientname,
			&size)) < 0)
		{
			printf("do_server %d: accept failed\n", __LINE__);
			return;
		}
        




        do_server_connection();





    }
}



int main(int argc, char *argv[])
{
#ifdef LOAD_COORDS
    do_debug_file();
#endif // LOAD_COORDS

#ifdef DO_TEST_PHOTOS
    do_poser(0);
#endif // DO_TEST_PHOTOS
    
#ifdef DO_SERVER
    do_server();
#endif // DO_SERVER
    
    return 0;
}






