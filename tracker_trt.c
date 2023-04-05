/*
 * tracker using body_25 for FP16
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
#include "NvInfer.h"
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
#include <unistd.h>

#define ENGINE_PORTRAIT "body25_240x160.engine"
#define ENGINE_LANDSCAPE "body25_144x256.engine"
//#define ENGINE "body25_128x224.engine"

// maximum humans to track
#define MAX_HUMANS 2

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <fcntl.h>
// video devices
#define DEVICE0 0
#define DEVICE1 4

#define VIDEO_BUFFERS 1



int decoded_w = 0;
int decoded_h = 0;
int current_input = 0;

// copy of the mmap image sent to the server
// 1 is owned by the server for JPEG compression
// buffer for JPEG decompression
uint8_t *hdmi_image[INPUT_IMAGES];
uint8_t **hdmi_rows[INPUT_IMAGES];

// storage for packet header, keypoints & compressed frame
uint8_t vijeo_buffer[HEADER_SIZE + 2 + MAX_HUMANS * BODY_PARTS * 4 + MAX_JPEG];

class Logger : public nvinfer1::ILogger
{
public:
    Logger(Severity severity = Severity::kWARNING)
    {
    }

    void log(Severity severity, const char* msg) noexcept
    {
        printf("%s\n", msg);
    }
};
Logger gLogger;


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

class Engine
{
public:
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    std::vector<nvinfer1::Dims> inputDims;
    std::vector<nvinfer1::Dims> outputDims;
    int batchSize;
    int numChannels;
// camera input
    int cam_w;
    int cam_h;
	void *hdmiCUDA = 0; 
// network dimensions
	int inputH;
	int inputW;
    int outputH;
    int outputW;
    std::vector<void*> cudaBuffers;
#define MAP_SIZE 78
#define PEAKS_W 128
#define PEAKS_H 3
#define MAX_PEAKS (PEAKS_W - 1)
    void *heatMapsBlobGPU;
    void *peaksBlobGPU;
    float *peaksBlobCPU;
    void *kernelGPU;
    void *bodyPartPairsGPU;
    op::Array<float> mFinalOutputCpu;
    void *pFinalOutputGpuPtr;
    void *mapIdxGpuPtr;
    op::Array<float> mPoseKeypoints;
    op::Array<float> mPoseScores;
    cudaStream_t cudaStream;


    std::size_t getSizeByDim(const nvinfer1::Dims& dims)
    {
	    std::size_t size = 1;
	    for (std::size_t i = 0; i < dims.nbDims; ++i)
	    {
		    size *= dims.d[i];
	    }
	    return size;
    }

    double resizeGetScaleFactor(int w1, int h1, int w2, int h2)
    {
        const auto ratioWidth = (double)(w2 - 1) / (double)(w1 - 1);
        const auto ratioHeight = (double)(h2 - 1) / (double)(h1 - 1);
        return MIN(ratioWidth, ratioHeight);
    }


    template<typename T>
    inline int positiveIntRound(const T a)
    {
        return int(a+0.5f);
    }


    void load(const char *path, int cam_w, int cam_h)
    {
        printf("Engine::load %d: Opening model %s\n", __LINE__, path);
        FILE *fd = fopen(path, "r");
        if(!fd)
        {
            printf("Engine::load %d: Couldn't open engine %s\n", __LINE__, path);
            exit(1);
        }
        fseek(fd, 0, SEEK_END);
        int len = ftell(fd);
        fseek(fd, 0, SEEK_SET);
        auto data = new char[len];
        int _ = fread(data, 1, len, fd);
        fclose(fd);

	    auto runtime = nvinfer1::createInferRuntime(gLogger);
	    engine = runtime->deserializeCudaEngine(data, len, nullptr);
        delete [] data;
        context = engine->createExecutionContext();
        
        cudaBuffers.resize(engine->getNbBindings());
	    for (size_t i = 0; i < engine->getNbBindings(); ++i)
	    {
		    auto bindingSize = getSizeByDim(engine->getBindingDimensions(i)) * 
                1 * 
                sizeof(float);
// cudaBuffers[0] is input
// cudaBuffers[1] is output
		    cudaMalloc(&cudaBuffers[i], bindingSize);
		    if(engine->bindingIsInput(i))
		    {
			    inputDims.emplace_back(engine->getBindingDimensions(i));
		    }
		    else
		    {
			    outputDims.emplace_back(engine->getBindingDimensions(i));
		    }
// 		    printf("Engine::load %d: binding=%s", 
//                 __LINE__,
//                 engine->getBindingName(i));
	    }

	    batchSize = inputDims[0].d[0];
	    numChannels = inputDims[0].d[1];
	    inputH = inputDims[0].d[2];
	    inputW = inputDims[0].d[3];
        outputH = outputDims[0].d[2];
        outputW = outputDims[0].d[3];
		printf("Engine::load %d: batchSize=%d numChannels=%d cam_w=%d cam_h=%d inputW=%d inputH=%d outputW=%d outputW=%d\n", 
            __LINE__,
            batchSize,
            numChannels,
            cam_w,
            cam_h,
            inputW,
            inputH,
            outputW,
            outputH);
        this->cam_w = cam_w;
        this->cam_h = cam_h;
        
	    cudaMalloc(&hdmiCUDA, HDMI_W * HDMI_H * 2); 
	    cudaStreamCreate(&cudaStream);
        cudaMalloc(&heatMapsBlobGPU, 
            outputDims[0].d[1] *
            inputW * 
            inputH * sizeof(float));
        cudaMalloc(&peaksBlobGPU, 
            BODY_PARTS * PEAKS_W * PEAKS_H * sizeof(float));
        peaksBlobCPU = (float*)malloc(BODY_PARTS * PEAKS_W * PEAKS_H * sizeof(float));
        cudaMalloc(&kernelGPU, 
            MAP_SIZE * inputW * inputH * sizeof(int));

        const auto& bodyPartPairs = op::getPosePartPairs(BODY_25);
        int numberBodyPartPairs = bodyPartPairs.size() / 2;
        int totalComputations = numberBodyPartPairs * MAX_PEAKS * MAX_PEAKS;
        cudaMalloc(&pFinalOutputGpuPtr,
            totalComputations * sizeof(float));
        mFinalOutputCpu.reset({(int)numberBodyPartPairs, MAX_PEAKS, MAX_PEAKS});
        cudaMalloc(&bodyPartPairsGPU,
            bodyPartPairs.size() * sizeof(unsigned int));
        cudaMemcpy(bodyPartPairsGPU, 
            &bodyPartPairs[0], 
            bodyPartPairs.size() * sizeof(unsigned int),
            cudaMemcpyHostToDevice);
        const auto& mapIdxOffset = op::getPoseMapIndex(BODY_25);
        // Update mapIdx
        const auto offset = (op::addBkgChannel(BODY_25) ? 1 : 0);
        auto mapIdx = mapIdxOffset;
        for (auto& i : mapIdx)
            i += (BODY_PARTS+offset);
        cudaMalloc(&mapIdxGpuPtr, 
            mapIdx.size() * sizeof(unsigned int));
        cudaMemcpy(mapIdxGpuPtr, 
            &mapIdx[0], 
            mapIdx.size() * sizeof(unsigned int),
            cudaMemcpyHostToDevice);
    }
    
    
    void process()
    {
        mPoseKeypoints.setTo(0);
#ifdef RAW_HDMI
// transfer input to GPU
	    cudaMemcpy(hdmiCUDA,  // dst
            hdmi_image[current_input], // src
            HDMI_W * HDMI_H * 2, 
            cudaMemcpyHostToDevice);
// convert to network size & RGB
		resizeAndNorm_yuv(hdmiCUDA,   // src
            (float*)cudaBuffers[0],  // dst
            HDMI_W, // srcW
            HDMI_H, // srcH
            inputW, // dstW
            inputH, // dstH
            cudaStream);
        cudaDeviceSynchronize();
        cudaError_t error = cudaGetLastError();
        if(error != 0)
            printf("Engine::process %d: %d %s\n", __LINE__, error, cudaGetErrorString(error) );
// write_test("/tmp/tracker2.ppm", cudaBuffers[0], numChannels, 1, inputW, inputH);
// 
// // DEBUG
// static int debug = 0;
// debug++;
// if(debug > 10) exit(0);

#endif

// Inference
	    context->enqueue(batchSize, 
            cudaBuffers.data(), 
            cudaStream, 
            nullptr);


        std::vector<std::array<int, 4>> mBottomSizes;
        std::array<int, 4> mTopSize = {
            outputDims[0].d[0], 
            outputDims[0].d[1], 
            inputH, 
            inputW
        };
        std::vector<float> mScaleRatios = { 1.0 };
        mBottomSizes.resize(1);
        mBottomSizes[0] = std::array<int, 4>{
            outputDims[0].d[0], 
            outputDims[0].d[1], 
            outputH, 
            outputW
        };
        std::vector<const float*> sourcePtrs(1);
        sourcePtrs[0] = (const float*)cudaBuffers[1];
// scale back up to input network size
        op::resizeAndMergeGpu((float*)heatMapsBlobGPU,  // dst
            sourcePtrs, // src
            mTopSize, // dst
            mBottomSizes, // src
            mScaleRatios);

        auto scaleProducerToNetInput = resizeGetScaleFactor( 
            cam_w, 
            cam_h,
            inputW, 
            inputH);
        int net_w = positiveIntRound(scaleProducerToNetInput * cam_w);
        int net_h = positiveIntRound(scaleProducerToNetInput * cam_h);
        auto mScaleNetToOutput = (float)resizeGetScaleFactor(
            net_w,
            net_h,
            cam_w,
            cam_h);
        auto nmsOffset = float(0.5/double(mScaleNetToOutput));
        op::Point<float> offset = { nmsOffset, nmsOffset };
        std::array<int, 4> targetSize = 
        {
            1, 
            BODY_PARTS, 
            PEAKS_W, 
            PEAKS_H 
        };
        std::array<int, 4> sourceSize = 
        {
            1, 
            MAP_SIZE, 
            inputH, 
            inputW 
        };

        op::nmsGpu(
            (float*)peaksBlobGPU, // dst
            (int*)kernelGPU, 
            (float*)heatMapsBlobGPU, // src
            (float)0.050000, // threshold
            targetSize,
            sourceSize, 
            offset);
// required copy
        cudaMemcpy(peaksBlobCPU, 
            peaksBlobGPU, 
            BODY_PARTS * PEAKS_W * PEAKS_H * sizeof(float), 
            cudaMemcpyDeviceToHost);

        op::connectBodyPartsGpu(
            mPoseKeypoints, 
            mPoseScores, 
            (float*)heatMapsBlobGPU, // GPU pointer
            (float*)peaksBlobCPU, // CPU pointer
            BODY_25, // poseModel
            op::Point<int>{inputW, inputH}, 
            MAX_PEAKS,  // maxPeaks
            (float).95, // interMinAboveThreshold,
            (float).05, // interThreshold, 
            3, // minSubsetCnt, 
            (float).4, // minSubsetScore, 
            (float).05, // defaultNmsThreshold,
            (float)mScaleNetToOutput, // scaleFactor, 
            false, // maximizePositives, 
            mFinalOutputCpu, // pairScoresCpu, 
            (float*)pFinalOutputGpuPtr, // pairScoresGpuPtr,
            (unsigned int*)bodyPartPairsGPU, // bodyPartPairsGpuPtr
            (unsigned int*)mapIdxGpuPtr, // mapIdxGpuPtr
            (float*)peaksBlobGPU); // peaksGpuPtr
    }
};

Engine landscape_engine;
Engine portrait_engine;


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
    int x1, y1, x2, y2;
// last detected size of the head for tilt tracking
    int head_size;
    zone_t zones[TOTAL_ZONES];
} body_t;

static body_t bodies[MAX_HUMANS];

// raw PWM values
float pan = PAN0;
float tilt = TILT0;
float start_pan = pan;
float start_tilt = tilt;
int pan_sign = 1;
int tilt_sign = 1;
int lens = LENS_15;
int landscape = 1;

static int servo_fd = -1;
static int frames = 0;
static FILE *ffmpeg_fd = 0;

int current_operation = STARTUP;
uint8_t error_flags = 0xff;

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

void stop_servos()
{
	if(servo_fd >= 0)
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
// write it a few times to defeat UART initialization glitches
    write_servos(1);
    usleep(100000);
    write_servos(1);
    usleep(100000);
    write_servos(1);
    usleep(100000);
    write_servos(1);
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

    fprintf(fd, "PAN %d\n", (int)start_pan);
    fprintf(fd, "TILT %d\n", (int)start_tilt);
    fprintf(fd, "PAN_SIGN %d\n", pan_sign);
    fprintf(fd, "TILT_SIGN %d\n", tilt_sign);
    fprintf(fd, "LENS %d\n", lens);
    fprintf(fd, "LANDSCAPE %d\n", landscape);

    fclose(fd);
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



void decompress_jpeg(uint8_t *picture_data, int picture_size)
{
    if(picture_data[0] != 0xff ||
        picture_data[1] != 0xd8 ||
        picture_data[2] != 0xff ||
        picture_data[3] != 0xdb)
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

    decoded_w = cinfo.output_width;
    decoded_h = cinfo.output_height;
//printf("decompress_jpeg %d w=%d h=%d\n", __LINE__, decoded_w, decoded_h);

	while(cinfo.output_scanline < decoded_h)
	{
		int num_scanlines = jpeg_read_scanlines(&cinfo, 
			&hdmi_rows[current_input][cinfo.output_scanline],
			decoded_h - cinfo.output_scanline);
	}

    jpeg_destroy_decompress(&cinfo);
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
    int humans = poseKeypoints.getSize(0);
    if(humans > MAX_HUMANS)
    {
        humans = MAX_HUMANS;
    }

// printf("do_feedback %d humans=%d w=%d h=%d\n", 
// __LINE__, 
// humans,
// w,
// h);


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


        if(bodyParts >= BODY_PARTS)
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

// focal point of all humans
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
    for(int human = 0; human < humans; human++)
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


    if(humans > 0)
    {
// average coords of all bodies
        total_x /= humans;
        total_y /= humans;

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

//printf("pan_change=%d tilt_change=%d\n", (int)pan_change, (int)tilt_change);
//    printf("pan=%d tilt=%d\n", (int)(pan - start_pan), (int)(tilt - start_tilt));

        write_servos(0);
    }
}


void do_tracker()
{
    cudaSetDevice(0);

    landscape_engine.load(ENGINE_LANDSCAPE, CAM_W, CAM_H);
    portrait_engine.load(ENGINE_PORTRAIT, CAM_H, CAM_H * 3 / 2);





    for(int i = 0; i < INPUT_IMAGES; i++)
    {
        hdmi_image[i] = new uint8_t[HDMI_W * HDMI_H * 2];
        hdmi_rows[i] = new uint8_t*[HDMI_H];
        for(int j = 0; j < HDMI_H; j++)
        {
            hdmi_rows[i][j] = hdmi_image[i] + j * HDMI_W * 2;
        }
    }

// YUV to RGB conversion from guicast
    cmodel_init();

#ifndef RAW_HDMI
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



    float fps = 0;
    struct timespec fps_time1;
    struct timespec feedback_time1;
    clock_gettime(CLOCK_MONOTONIC, &fps_time1);
    clock_gettime(CLOCK_MONOTONIC, &feedback_time1);
    int frame_count = 0;

    int current_device = DEVICE0;
    int fd = -1;
    unsigned char *mmap_buffer[VIDEO_BUFFERS];

    while(1)
    {
        if(fd < 0)
        {
            char string[TEXTLEN];
            sprintf(string, "/dev/video%d", current_device);
            printf("do_tracker %d opening %s\n", 
                __LINE__, 
                string);

            fd = open(string, O_RDWR);
            if(fd < 0)
            {
                if(!(error_flags & VIDEO_DEVICE_ERROR))
                {
                    printf("do_tracker %d: failed to open %s\n",
                        __LINE__,
                        string);
                    error_flags |= VIDEO_DEVICE_ERROR;
                    send_error();
                }
                sleep(1);
                current_device++;
                if(current_device > DEVICE1)
                    current_device = 0;
            }
            else
            {
                printf("do_tracker %d: opened %s\n",
                    __LINE__,
                    string);
                if((error_flags & VIDEO_DEVICE_ERROR))
                {
                    error_flags &= ~VIDEO_DEVICE_ERROR;
                    send_error();
                }

                struct v4l2_format v4l2_params;
                v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                ioctl(fd, VIDIOC_G_FMT, &v4l2_params);

                v4l2_params.fmt.pix.width = HDMI_W;
                v4l2_params.fmt.pix.height = HDMI_H;

#ifndef RAW_HDMI
                v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#else
                v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif
                if(ioctl(fd, VIDIOC_S_FMT, &v4l2_params) < 0)
                {
                    printf("do_tracker %d: VIDIOC_S_FMT failed\n",
                        __LINE__);
                }


                struct v4l2_requestbuffers requestbuffers;
                requestbuffers.count = VIDEO_BUFFERS;
                requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                requestbuffers.memory = V4L2_MEMORY_MMAP;
                if(ioctl(fd, VIDIOC_REQBUFS, &requestbuffers) < 0)
                {
                    printf("do_tracker %d: VIDIOC_REQBUFS failed\n",
                        __LINE__);
                }
                else
                {
                    for(int i = 0; i < VIDEO_BUFFERS; i++)
                    {
                        struct v4l2_buffer buffer;
                        buffer.type = requestbuffers.type;
                        buffer.index = i;

                        if(ioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0)
				        {
					        printf("do_tracker %d: VIDIOC_QUERYBUF failed\n",
                                __LINE__);
				        }
                        else
                        {
                            mmap_buffer[i] = (unsigned char*)mmap(NULL,
					            buffer.length,
					            PROT_READ | PROT_WRITE,
					            MAP_SHARED,
					            fd,
					            buffer.m.offset);
                            printf("do_tracker %d: allocated buffer size=%d\n",
                                __LINE__,
                                buffer.length);
                            if(ioctl(fd, VIDIOC_QBUF, &buffer) < 0)
                            {
                                printf("do_tracker %d: VIDIOC_QBUF failed\n",
                                    __LINE__);
                            }
                        }
                    }
                }

                int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	            if(ioctl(fd, VIDIOC_STREAMON, &streamon_arg) < 0)
                {
		            printf("do_tracker %d: VIDIOC_STREAMON failed\n",
                        __LINE__);
                }
            }
        }
        
        
        
        if(fd >= 0)
        {
            struct v4l2_buffer buffer;
		    bzero(&buffer, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buffer.memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd, VIDIOC_DQBUF, &buffer) < 0)
            {
                printf("do_tracker %d: VIDIOC_DQBUF failed\n",
                    __LINE__);
                if((error_flags & VIDEO_BUFFER_ERROR) == 0)
                {
                    error_flags |= VIDEO_BUFFER_ERROR;
                    send_error();
                }
                close(fd);
                fd = -1;
                sleep(1);
            }
            else
            {
                if((error_flags & VIDEO_BUFFER_ERROR))
                {
                    error_flags &= ~VIDEO_BUFFER_ERROR;
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
                ioctl(fd, VIDIOC_QBUF, &buffer);
#endif // RAW_HDMI

                Engine *engine;
                if(landscape)
                    engine = &landscape_engine;
                else
                    engine = &portrait_engine;

                engine->process();

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


                if(current_input2 < 0)
                {
// send keypoints
                    int animals = engine->mPoseKeypoints.getSize(0);
                    if(animals > MAX_HUMANS) animals = MAX_HUMANS;
//printf("do_tracker %d animals=%d\n", __LINE__, animals);
                    int offset = HEADER_SIZE;
                    int max_x = 0;
                    int max_y = 0;

                    vijeo_buffer[offset++] = animals;
                    vijeo_buffer[offset++] = 0;
                    for(int i = 0; i < animals; i++)
                    {
                        for(int j = 0; j < BODY_PARTS; j++)
                        {
                            int x = (int)engine->mPoseKeypoints[{ i, j, 0 }];
                            int y = (int)engine->mPoseKeypoints[{ i, j, 1 }];
                            vijeo_buffer[offset++] = x & 0xff;
                            vijeo_buffer[offset++] = (x >> 8) & 0xff;
                            vijeo_buffer[offset++] = y & 0xff;
                            vijeo_buffer[offset++] = (y >> 8) & 0xff;
                            if(x > max_x) max_x = x;
                            if(y > max_y) max_y = y;
                        }
                    }
                    send_vijeo(current_input, offset - HEADER_SIZE);
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
            }
        }
    }
}

int main(int argc, char *argv[])
{
    load_defaults();
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












