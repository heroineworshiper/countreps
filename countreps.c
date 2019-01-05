// To build countreps:
// 
// LD_LIBRARY_PATH=lib/ make
// 
// To run it, specify the library path:
// 
// LD_LIBRARY_PATH=lib/ ./countreps
// 
// install kernel module:
// insmod ./4.9.39/kernel/drivers/video/nvidia-uvm.ko


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

#include <vector>

#define INPATH "test_input"
#define OUTPATH "test_output"
#define MODELS "../openpose/models/"
#define TEXTLEN 1024
#define DEBUG_COORDS "debug_coords"


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


using std::vector;


class Process : public op::WorkerConsumer<std::shared_ptr<std::vector<op::Datum>>>
{
public:
// the current frame number
    int frames = 0;
// the reps detected
    int reps = 0;
    int prev_reps = 0;
    int pose = UNKNOWN_POSE;
    int prev_pose = UNKNOWN_POSE;
// the current exercise
    int plan_line = 0;
    int exercise = plan[plan_line].exercise;
    
    void initializationOnThread() {}

// get raw data from openpose
    void workConsumer(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
    {
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            
            
            const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
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
            
            
            frames++;
        }
    }
    
    FILE *debug_fd = 0;
    void process(coord_t *coords)
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
//        printf("%s\n", string2);
        fflush(debug_fd);
    }


// process just based on the coordinates
    void process2(coord_t *coords)
    {
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

    int get_exercise_pose(coord_t *coords)
    {
// Y distances
        float butt_foot1_dist = fabs(coords[MODEL_ANKLE1].y - 
            coords[MODEL_BUTT].y);
        float butt_foot2_dist = fabs(coords[MODEL_ANKLE2].y - 
            coords[MODEL_BUTT].y);
        float max_butt_foot_dist = MAX(butt_foot1_dist, butt_foot2_dist);
        int have_foot_dist = (coords[MODEL_ANKLE1].y > 0.001 &&
            coords[MODEL_ANKLE2].y > 0.001);
        
        float butt_neck_dist = fabs(coords[MODEL_NECK].y - 
            coords[MODEL_BUTT].y);

// angles
        float butt_neck = get_angle(coords, MODEL_BUTT, MODEL_NECK);
        float butt_knee1 = get_angle(coords, MODEL_BUTT, MODEL_KNEE1);
        float butt_knee2 = get_angle(coords, MODEL_BUTT, MODEL_KNEE2);
        float elbow1_neck = get_angle(coords, MODEL_ELBOW1, MODEL_SHOULDER1);
        float elbow2_neck = get_angle(coords, MODEL_ELBOW2, MODEL_SHOULDER2);
        float max_knee = 0;
        int have_knee1 = (coords[MODEL_KNEE1].x > 0.001);
        int have_knee2 = (coords[MODEL_KNEE2].x > 0.001);
        if(!have_knee1)
        {
            max_knee = butt_knee2;
        }
        else
        if(!have_knee2)
        {
            max_knee = butt_knee1;
        }
        else
        {
            max_knee = MAX(butt_knee1, butt_knee2);
        }
        
        
        
        int result = UNKNOWN_POSE;


        switch(exercise)
        {
            case SITUPS:
                if(butt_neck > TORAD(135) &&
                    max_knee > TORAD(25))
                {
                    result = SITUP_DOWN;
                }
                else
                if(butt_neck < TORAD(135) &&
                    butt_neck > TORAD(90) &&
                    max_knee > TORAD(25))
                {
                    result = SITUP_UP;
                }
                break;
            
            case PUSHUPS:
// only test pushups facing right, because of occluded left elbow
                if(butt_neck > TORAD(0) &&
                    butt_neck < TORAD(45) &&
                    elbow2_neck > TORAD(60) &&
                    (max_knee > TORAD(135) ||
                    max_knee < TORAD(-90)))
                {
                    result = PUSHUP_UP;
                }
                else
                if(butt_neck > TORAD(0) &&
                    butt_neck < TORAD(45) &&
                    elbow2_neck < TORAD(25) &&
                    (max_knee > TORAD(135) ||
                    max_knee < TORAD(-90)))
                {
                    result = PUSHUP_DOWN;
                }
                break;
            
            case HIP_FLEX:
// hip flex or squat flex
                if(butt_neck > TORAD(0) &&
                    butt_neck < TORAD(120) &&
                    max_knee < TORAD(-45) &&
                    max_knee > TORAD(-120))
                {
                    result = STANDING;
                }
// hip flex.  Can't differentiate side.
                else
                if(butt_neck > TORAD(60) &&
                    butt_neck < TORAD(120) &&
                    max_knee > TORAD(-45) &&
                    max_knee < TORAD(30) &&
                    have_foot_dist &&
                    max_butt_foot_dist > butt_neck_dist / 2)
                {
                    result = HIP_UP;
                }
                break;
            
            case SQUAT:
// hip flex or squat flex
                if(butt_neck > TORAD(0) &&
                    butt_neck < TORAD(120) &&
                    max_knee < TORAD(-25) &&
                    max_knee > TORAD(-120))
                {
                    result = STANDING;
                }
                else
// squat flex
                if(butt_neck > TORAD(0) &&
                    butt_neck < TORAD(90) &&
                    max_knee > TORAD(25) &&
                    max_knee < TORAD(90))
                {
                    result = SQUAT_DOWN;
                }
                break;
        }
        
        printf("Process.get_exercise_pose %d: frame=%d back=%d knees=%d %d elbows=%d %d pose=%d\n",
            __LINE__,
            frames,
            (int)TODEG(butt_neck),
            (int)TODEG(butt_knee1),
            (int)TODEG(butt_knee2),
            (int)TODEG(elbow1_neck),
            (int)TODEG(elbow2_neck),
            result);

        return result;
        

    }

// get angle between 2 body parts
    float get_angle(coord_t *coords, int body_part1, int body_part2)
    {
        return -atan2(coords[body_part2].y - coords[body_part1].y,
            coords[body_part2].x - coords[body_part1].x);
    }

// increment the reps counter
    int calculate_reps()
    {
        switch(exercise)
        {
            case SITUPS:
                if(prev_pose == SITUP_DOWN && pose == SITUP_UP)
                {
                    reps++;
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
                    reps++;
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
                    reps++;
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
                    reps++;
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




int main(int argc, char *argv[])
{
#if 1
// process a debug file instead of using openpose
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
        processor.frames = atoi(ptr);

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

        processor.process2(coords);
    }
    
 


// DEBUG
exit(0);
#endif // 0


    
// start the pose estimator
    
    op::Wrapper opWrapper;
    opWrapper.setWorker(op::WorkerType::Output, // workerType
        std::make_shared<Process>(), // worker
        true); // const bool workerOnNewThread

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


// Producer (use default to disable any input)
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



// GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui
    {
        op::flagsToDisplayMode(-1, false), 
        true, 
        false, // FLAGS_fullscreen
    };
    opWrapper.configure(wrapperStructGui);

    
    opWrapper.exec();
    
    
    
    
    
    return 0;
}






