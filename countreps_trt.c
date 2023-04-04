/*
 * countreps using the tensorrt back end
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
// ./countreps_trt


// OpenPose dependencies
#include <cuda.h>
#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include <memory>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include "jpeglib.h"
#include <setjmp.h>


#include "gui.h"

// test photos go here
#define INPATH "test_input"
// output photos go here
#define OUTPATH "test_output"
// coordinates go here
#define DEBUG_COORDS "debug_coords"

// read frames from test_input
//#define READ_INPUT

// read frames from video4linux
#define READ_V4L2
#ifdef READ_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <fcntl.h>
// video devices
#define DEVICE0 0
#define DEVICE1 4
//#define INPUT_W 1280
//#define INPUT_H 720
#define INPUT_W 640
#define INPUT_H 360
#define VIDEO_BUFFERS 1
#endif


int decoded_w = 0;
int decoded_h = 0;
unsigned char *input_image = 0;
unsigned char **input_rows = 0;




// read the coordinates from DEBUG_COORDS & print the results without
// doing anything else
//#define LOAD_COORDS

// save coordinates to DEBUG_COORDS
#define SAVE_COORDS

// save frames in test_input
//#define SAVE_INPUT

// save output frames in test_output
//#define SAVE_OUTPUT

// use the body_25 model ported from caffe
#define USE_OPENPOSE

// The engine
#ifdef USE_OPENPOSE
    #include "point.hpp"
    #include "resize.h"
    #include "resizeAndMergeBase.hpp"
    #include "nmsBase.hpp"
    #include "array.hpp"
    #include "poseParameters.hpp"
    #include "bodyPartConnectorBase.hpp"
//    #define ENGINE "body25_128x224.engine"
    #define ENGINE "body25_144x256.engine"
//    #define ENGINE "body25_144x256_int8.engine"


#else
// trt openpose class
    #include "Openpose.h"
    #define ENGINE "trt_pose_fp16.engine"
#endif


#define TEXTLEN 1024
#define TO_MS(x) ((x).tv_sec * 1000 + (x).tv_usec / 1000)
// max frame rate to limit power usage
//#define FPS 30
#define MIN(x, y) ((x) < (y) ? (x) : (y))

// error bits
#define VIDEO_DEVICE_ERROR 1
#define VIDEO_BUFFER_ERROR 2
#define SERVO_ERROR 4
uint8_t error_flags;

// reps must be separated by this many frames
#define DEBOUNCE 4



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





void lock_sema(void *ptr)
{
#ifndef __clang__
    sem_t *sema = (sem_t*)ptr;
    sem_wait(sema);
#else
    dispatch_semaphore_t *sema = (dispatch_semaphore_t*)ptr;
    dispatch_semaphore_wait(*sema, DISPATCH_TIME_FOREVER);
#endif
}

void unlock_sema(void *ptr)
{
#ifndef __clang__
    sem_t *sema = (sem_t*)ptr;
    sem_post(sema);
#else
    dispatch_semaphore_t *sema = (dispatch_semaphore_t*)ptr;
    dispatch_semaphore_signal(*sema);
#endif
}







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


std::size_t getSizeByDim(const nvinfer1::Dims& dims)
{
	std::size_t size = 1;
	for (std::size_t i = 0; i < dims.nbDims; ++i)
	{
		size *= dims.d[i];
	}
	return size;
}

inline void* safeCudaMalloc(size_t memSize)
{
	void* deviceMem;
	cudaMalloc(&deviceMem, memSize);
	if (deviceMem == nullptr)
	{
		std::cerr << "Out of memory" << std::endl;
		exit(1);
	}
	return deviceMem;
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

typedef struct my_jpeg_error_mgr* my_jpeg_error_ptr;
struct my_jpeg_error_mgr my_jpeg_error;

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
  	my_jpeg_error_ptr mjpegerr = (my_jpeg_error_ptr) cinfo->err;

printf("my_jpeg_error_exit %d\n", __LINE__);
/* Always display the message. */
/* We could postpone this until after returning, if we chose. */
  	(*cinfo->err->output_message) (cinfo);

/* Return control to the setjmp point */
  	longjmp(mjpegerr->setjmp_buffer, 1);
}

void update_error()
{
}

// write a buffer as a PPM image
void write_test2(const char *path, 
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


void decompress_jpeg(uint8_t *picture_data, int picture_size)
{
	struct jpeg_decompress_struct cinfo;
	struct my_jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
    my_jpeg_error.pub.error_exit = my_jpeg_error_exit;
    my_jpeg_error.pub.output_message = my_jpeg_output;
	jpeg_create_decompress(&cinfo);
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
    if(!input_image)
    {
printf("decompress_jpeg %d w=%d h=%d\n", __LINE__, decoded_w, decoded_h);
        input_image = new uint8_t[decoded_w * 3 * decoded_h];
        input_rows = new uint8_t*[decoded_h];
        for(int i = 0; i < decoded_h; i++)
        {
            input_rows[i] = input_image + decoded_w * i * 3;
        }
    }

	while(cinfo.output_scanline < decoded_h)
	{
		int num_scanlines = jpeg_read_scanlines(&cinfo, 
			&input_rows[cinfo.output_scanline],
			decoded_h - cinfo.output_scanline);
	}

    jpeg_destroy_decompress(&cinfo);
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

#define PROFILE_PRINT \
    gettimeofday(&profile_time2, 0); \
    printf("do_poser %d: time=%dms\n", \
        __LINE__, \
        (int)(TO_MS(profile_time2) - TO_MS(profile_time1))); \
    profile_time1 = profile_time2;



void do_poser()
{
    cudaSetDevice(0);
    Logger gLogger;

    printf("Opening model %s\n", ENGINE);
    FILE *engine_fd = fopen(ENGINE, "r");
    if(!engine_fd)
    {
        printf("Couldn't open engine %s\n", ENGINE);
        exit(1);
    }
    fseek(engine_fd, 0, SEEK_END);
    int len = ftell(engine_fd);
    fseek(engine_fd, 0, SEEK_SET);
    auto data = new char[len];
    int _ = fread(data, 1, len, engine_fd);
    fclose(engine_fd);
    
	auto runtime = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(gLogger));
	auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(data, len, nullptr));
    delete [] data;
    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    std::vector<void*> cudaBuffers;
	cudaBuffers.resize(engine->getNbBindings());

	std::vector<nvinfer1::Dims> inputDims;
	std::vector<nvinfer1::Dims> outputDims;
	for (size_t i = 0; i < engine->getNbBindings(); ++i)
	{
		auto bindingSize = getSizeByDim(engine->getBindingDimensions(i)) * 
            1 * 
            sizeof(float);
// cudaBuffers[0] is input
// cudaBuffers[1] is output
		cudaMalloc(&cudaBuffers[i], bindingSize);
		if (engine->bindingIsInput(i))
		{
			inputDims.emplace_back(engine->getBindingDimensions(i));
		}
		else
		{
			outputDims.emplace_back(engine->getBindingDimensions(i));
		}
		std::cout << "Binding Name: " << engine->getBindingName(i) << std::endl;
	}

	int batchSize = inputDims[0].d[0];
	int numChannels = inputDims[0].d[1];
	int inputH = inputDims[0].d[2];
	int inputW = inputDims[0].d[3];
    int outputH = outputDims[0].d[2];
    int outputW = outputDims[0].d[3];
#ifdef USE_OPENPOSE
#define BODY_PARTS 25
#define MAP_SIZE 78
    void *heatMapsBlobGPU = 0;
    void *peaksBlobGPU = 0;
    float *peaksBlobCPU = 0;
    void *kernelGPU = 0;
    void *bodyPartPairsGPU = 0;
    op::Array<float> mFinalOutputCpu;
    void *pFinalOutputGpuPtr = 0;
    void *mapIdxGpuPtr = 0;
    op::Array<float> mPoseKeypoints;
    op::Array<float> mPoseScores;
#endif

//     printf("do_poser %d: batchSize=%d inputDims=%d outputDims=%d\n",
//         __LINE__,
//         batchSize,
//         (int)inputDims.size(),
//         (int)outputDims.size());
//     for(int j = 0; j < inputDims.size(); j++)
//     {
//     	for (int i = 0; i < inputDims[j].nbDims; ++i)
//     	{
//             printf("do_poser %d: inputDims[%d][%d]=%d\n", 
//                 __LINE__, 
//                 j, 
//                 i, 
//                 (int)inputDims[j].d[i]);
//     	}
//     }
// 
//     for(int j = 0; j < outputDims.size(); j++)
//     {
// 	    for (int i = 0; i < outputDims[j].nbDims; ++i)
// 	    {
//             printf("do_poser %d: outputDims[%d][%d]=%d\n", 
//                 __LINE__, 
//                 j, 
//                 i, 
//                 outputDims[j].d[i]);
// 	    }
//     }

	printf("Model input shape: batchSize=%d channels=%d w=%d h=%d size=%d\n",
        batchSize,
        numChannels,
        inputW,
        inputH,
        (int)getSizeByDim(inputDims[0]));

#ifndef USE_OPENPOSE
	std::vector<float> cpuCmapBuffer;
	std::vector<float> cpuPafBuffer;
	cpuCmapBuffer.resize(getSizeByDim(outputDims[0]) * batchSize);
	cpuPafBuffer.resize(getSizeByDim(outputDims[1]) * batchSize);
    Openpose openpose(outputDims[0]);
#endif



// max input image shape
	void *cudaFrame = 0; 

    cudaStream_t cudaStream;
	cudaStreamCreate(&cudaStream);


// get input from video4linux2
    float fps = 0;
    struct timeval time1;
//    struct timespec fps_time1;
    gettimeofday(&time1, 0);
//    struct timeval profile_time1;
//    struct timeval profile_time2;
//    gettimeofday(&profile_time1, 0);
    int verbose = 0;
    int frame_count = 0;

#ifdef READ_V4L2
    printf("do_poser %d using V4L2\n", __LINE__);
	cudaFrame = safeCudaMalloc(INPUT_W * INPUT_H * numChannels * sizeof(uchar)); 
    int current_device = DEVICE0;
    int fd = -1;
    unsigned char *mmap_buffer[VIDEO_BUFFERS];
#endif // READ_V4L2

    while(1)
    {
#ifdef READ_V4L2
        if(fd < 0)
        {
            char string[TEXTLEN];
            sprintf(string, "/dev/video%d", current_device);
            if(verbose) printf("do_poser %d opening %s\n", 
                __LINE__, 
                string);

            fd = open(string, O_RDWR);
            if(fd < 0)
            {
                if(!(error_flags & VIDEO_DEVICE_ERROR))
                {
                    printf("do_poser %d: failed to open %s\n",
                        __LINE__,
                        string);
                    error_flags |= VIDEO_DEVICE_ERROR;
                    update_error();
                }
                sleep(1);
                current_device++;
                if(current_device > DEVICE1)
                    current_device = 0;
            }
            else
            {
                printf("do_poser %d: opened %s\n",
                    __LINE__,
                    string);
                if((error_flags & VIDEO_DEVICE_ERROR))
                {
                    error_flags &= ~VIDEO_DEVICE_ERROR;
                    update_error();
                }

                struct v4l2_format v4l2_params;
                v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                ioctl(fd, VIDIOC_G_FMT, &v4l2_params);

                v4l2_params.fmt.pix.width = INPUT_W;
                v4l2_params.fmt.pix.height = INPUT_H;

                v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
                if(ioctl(fd, VIDIOC_S_FMT, &v4l2_params) < 0)
                {
                    printf("do_poser %d: VIDIOC_S_FMT failed\n",
                        __LINE__);
                }


                struct v4l2_requestbuffers requestbuffers;
                requestbuffers.count = VIDEO_BUFFERS;
                requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                requestbuffers.memory = V4L2_MEMORY_MMAP;
                if(ioctl(fd, VIDIOC_REQBUFS, &requestbuffers) < 0)
                {
                    printf("do_poser %d: VIDIOC_REQBUFS failed\n",
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
					        printf("do_poser %d: VIDIOC_QUERYBUF failed\n",
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
                            if(verbose) printf("do_poser %d: allocated buffer size=%d\n",
                                __LINE__,
                                buffer.length);
                            if(ioctl(fd, VIDIOC_QBUF, &buffer) < 0)
                            {
                                printf("do_poser %d: VIDIOC_QBUF failed\n",
                                    __LINE__);
                            }
                        }
                    }
                }

                int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	            if(ioctl(fd, VIDIOC_STREAMON, &streamon_arg) < 0)
                {
		            printf("do_poser %d: VIDIOC_STREAMON failed\n",
                        __LINE__);
                }
//                clock_gettime(CLOCK_MONOTONIC, &fps_time1);
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
                printf("do_poser %d: VIDIOC_DQBUF failed\n",
                    __LINE__);
                if((error_flags & VIDEO_BUFFER_ERROR) == 0)
                {
                    error_flags |= VIDEO_BUFFER_ERROR;
                    update_error();
                }
                close(fd);
                delete [] input_image;
                delete [] input_rows;
                input_image = 0;
                input_rows = 0;
                fd = -1;
                sleep(1);
            }
            else
            {
                if((error_flags & VIDEO_BUFFER_ERROR))
                {
                    error_flags &= ~VIDEO_BUFFER_ERROR;
                    update_error();
                }

// discard if it arrived too soon
// TODO: this is broken or it adds too much latency
//                 struct timespec fps_time2;
//                 clock_gettime(CLOCK_MONOTONIC, &fps_time2);
//                 double delta = 
//                     (double)((fps_time2.tv_sec * 1000 + fps_time2.tv_nsec / 1000000) -
//                     (fps_time1.tv_sec * 1000 + fps_time1.tv_nsec / 1000000)) / 1000;
//                if(delta >= 1.0 / FPS)
//                if(1)
//                {
// decompress it
//                    fps_time1 = fps_time2;

                    decompress_jpeg(mmap_buffer[buffer.index], buffer.bytesused);

// release the buffer
                    ioctl(fd, VIDIOC_QBUF, &buffer);

// rotate it 180
                    for(int y1 = 0; y1 < decoded_h / 2; y1++)
                    {
                        int y2 = decoded_h - 1 - y1;
                        uint8_t *row1 = input_rows[y1];
                        uint8_t *row2 = input_rows[y2];
                        for(int x1 = 0; x1 < decoded_w; x1++)
                        {
                            int x2 = decoded_w - 1 - x1;
                            uint8_t *col1 = &row1[x1 * 3];
                            uint8_t *col2 = &row2[x2 * 3];
                            uint8_t temp;
                            temp = col1[0];
                            col1[0] = col2[0];
                            col2[0] = temp;
                            temp = col1[1];
                            col1[1] = col2[1];
                            col2[1] = temp;
                            temp = col1[2];
                            col1[2] = col2[2];
                            col2[2] = temp;
                        }
                    }
#endif // READ_V4L2

#ifdef READ_INPUT
                    printf("do_poser %d reading input file\n", __LINE__);
                    FILE *fd = fopen("test_input/jew1.jpg", "r");
                    fseek(fd, 0, SEEK_END);
                    int picture_size = ftell(fd);
                    fseek(fd, 0, SEEK_SET);
                    uint8_t *picture_data = (uint8_t*)malloc(picture_size);
                    int _ = fread(picture_data, 1, picture_size, fd);
                    fclose(fd);
                    decompress_jpeg(picture_data, picture_size);
                    free(picture_data);
	                if(!cudaFrame)
                        cudaFrame = safeCudaMalloc(decoded_w * decoded_h * numChannels); 
#endif


//                    gettimeofday(&profile_time1, 0);
                    mPoseKeypoints.setTo(0);

// transfer input to GPU
	                cudaMemcpy(cudaFrame,  // dst
                        input_image, // src
                        decoded_w * decoded_h * numChannels, 
                        cudaMemcpyHostToDevice);
// resize it & transpose
#ifdef USE_OPENPOSE
		            resizeAndNorm_openpose(cudaFrame,   // src
                        (float*)cudaBuffers[0],  // dst
                        decoded_w, 
                        decoded_h, 
                        inputW, 
                        inputH, 
                        cudaStream);
#else
		            resizeAndNorm(cudaFrame,   // src
                        (float*)cudaBuffers[0],  // dst
                        decoded_w, 
                        decoded_h, 
                        inputW, 
                        inputH, 
                        cudaStream);
#endif


//                    write_test2("/tmp/trt1.ppm", cudaBuffers[0], numChannels, 1, inputW, inputH);

// Inference
	                context->enqueue(batchSize, 
                        cudaBuffers.data(), 
                        cudaStream, 
                        nullptr);
//                    PROFILE_PRINT

#ifdef USE_OPENPOSE
//                    poseExtractor(cudaBuffers[1]);


//                    write_test2("/tmp/trt2.ppm", cudaBuffers[1], 13, 6, outputW, outputH);



                    if(!heatMapsBlobGPU)
                        cudaMalloc(&heatMapsBlobGPU, 
                            outputDims[0].d[1] *
                            inputW * 
                            inputH * sizeof(float));

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

//                    write_test2("/tmp/trt3.ppm", heatMapsBlobGPU, 13, 6, inputW, inputH);

//                    PROFILE_PRINT


// non maximum suppression
#define PEAKS_W 128
#define PEAKS_H 3
                    if(!peaksBlobGPU)
                    {
                        cudaMalloc(&peaksBlobGPU, 
                            BODY_PARTS * PEAKS_W * PEAKS_H * sizeof(float));
                        peaksBlobCPU = (float*)malloc(BODY_PARTS * PEAKS_W * PEAKS_H * sizeof(float));
                    }


                    auto scaleProducerToNetInput = resizeGetScaleFactor( 
                        decoded_w, 
                        decoded_h,
                        inputW, 
                        inputH);
                    int net_w = positiveIntRound(scaleProducerToNetInput * decoded_w);
                    int net_h = positiveIntRound(scaleProducerToNetInput * decoded_h);
                    auto mScaleNetToOutput = (float)resizeGetScaleFactor(
                        net_w,
                        net_h,
                        decoded_w,
                        decoded_h);
                    auto nmsOffset = float(0.5/double(mScaleNetToOutput));

// printf("do_poser %d scaleProducerToNetInput=%f mScaleNetToOutput=%f nmsOffset=%f\n", 
// __LINE__, 
// scaleProducerToNetInput,
// mScaleNetToOutput, 
// nmsOffset);

                    op::Point<float> offset = { nmsOffset, nmsOffset };
                    if(!kernelGPU)
                        cudaMalloc(&kernelGPU, 
                            MAP_SIZE * inputW * inputH * sizeof(int));

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

//                    PROFILE_PRINT
                    op::nmsGpu(
                        (float*)peaksBlobGPU, // dst
                        (int*)kernelGPU, 
                        (float*)heatMapsBlobGPU, // src
                        (float)0.050000, // threshold
                        targetSize,
                        sourceSize, 
                        offset);

//write_test2("/tmp/trt4.ppm", peaksBlobGPU, 1, BODY_PARTS, PEAKS_W, PEAKS_H);

//                    PROFILE_PRINT
// required copy
                    cudaMemcpy(peaksBlobCPU, 
                        peaksBlobGPU, 
                        BODY_PARTS * PEAKS_W * PEAKS_H * sizeof(float), 
                        cudaMemcpyDeviceToHost);
//                    PROFILE_PRINT
// for(int i = 0; i < BODY_PARTS; i++)
// {
//     for(int j = 0; j < 5; j++)
//     {
//         printf("%.2f ", peaksBlobCPU[i * 128 * 3 + j]);
//     }
//     printf("\n");
// }

                    int maxPeaks = PEAKS_W - 1;
                    const auto& bodyPartPairs = op::getPosePartPairs(BODY_25);
                    int numberBodyPartPairs = bodyPartPairs.size() / 2;
                    int totalComputations = numberBodyPartPairs * maxPeaks * maxPeaks;
                    

                    if(!bodyPartPairsGPU)
                    {
                        cudaMalloc(&pFinalOutputGpuPtr,
                            totalComputations * sizeof(float));
                        mFinalOutputCpu.reset({(int)numberBodyPartPairs, maxPeaks, maxPeaks});
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

// printf("do_poser %d totalComputations=%d\n", __LINE__, totalComputations);
// for(int i = 0; i < bodyPartPairs.size(); i++)
// printf("bodyPartPairs[%d]=%d\n", i, bodyPartPairs[i]);
// for(int i = 0; i < mapIdx.size(); i++)
// printf("mapIdx[%d]=%d\n", i, mapIdx[i]);

                    }

//                    PROFILE_PRINT
                    op::connectBodyPartsGpu(
                        mPoseKeypoints, 
                        mPoseScores, 
                        (float*)heatMapsBlobGPU, // GPU pointer
                        (float*)peaksBlobCPU, // CPU pointer
                        BODY_25, // poseModel
                        op::Point<int>{inputW, inputH}, 
                        maxPeaks,  // maxPeaks
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


// for(int i = 0; i < totalComputations; i++)
// if(mFinalOutputCpu[i] != -1) printf("i=%d mFinalOutputCpu=%f\n", i, mFinalOutputCpu[i]);
// 
// printf("do_poser %d mPoseKeypoints.size=%d\n",
// __LINE__,
// mPoseKeypoints.getSize(0));
// for(int i = 0; i < BODY_PARTS; i++)
// {
//     printf("part=%d %d %d\n", 
//         i, 
//         (int)mPoseKeypoints[{ 0, i, 0 }],
//         (int)mPoseKeypoints[{ 0, i, 1 }]);
// }

//                    PROFILE_PRINT





#else // USE_OPENPOSE

// Copy results from GPU to CPU
	                cudaMemcpy(cpuCmapBuffer.data(), 
                        (float*)cudaBuffers[1], 
                        cpuCmapBuffer.size() * sizeof(float), 
                        cudaMemcpyDeviceToHost);
	                cudaMemcpy(cpuPafBuffer.data(), 
                        (float*)cudaBuffers[2], 
                        cpuPafBuffer.size() * sizeof(float), 
                        cudaMemcpyDeviceToHost);
                    openpose.detect(cpuCmapBuffer.data(), 
                        cpuPafBuffer.data(), 
                        input_image,
                        decoded_w,
                        decoded_h);
#endif // !USE_OPENPOSE


// draw on GUI
                    update_gui(input_image, 
                        decoded_w, 
                        decoded_h, 
                        0, 
                        0,
                        fps,
                        1);
// Xlib conflicts with openpose so translate array
                    for(int i = 0; i < mPoseKeypoints.getSize(0); i++)
                    {
                        int xy_array[BODY_PARTS * 2];
                        for(int j = 0; j < BODY_PARTS; j++)
                        {
                            xy_array[j * 2 + 0] = (int)mPoseKeypoints[{ i, j, 0 }];
                            xy_array[j * 2 + 1] = (int)mPoseKeypoints[{ i, j, 1 }];
                        }
                        draw_body(xy_array, decoded_w, decoded_h);
                    }
                    flash_gui();
//                 }
//                 else
//                 {
// // release the buffer
//                     ioctl(fd, VIDIOC_QBUF, &buffer);
//                 }

                frame_count++;
                struct timeval time2;
                gettimeofday(&time2, 0);
                int64_t diff = TO_MS(time2) - TO_MS(time1);
                if(diff >= 1000)
                {
                    fps = (double)frame_count * 1000 / diff;
                    printf("do_poser %d: FPS: %f\n",
                        __LINE__,
                        fps);
                    frame_count = 0;
                    time1 = time2;
                }

#ifdef READ_V4L2
            } // VIDIOC_DQBUF
        } // fd >= 0
#endif


#ifdef READ_INPUT
        printf("Press enter to quit\n");
        fgetc(stdin);
        exit(0);
#endif

    }


// Get frames from test_input
#ifdef READ_INPUT
    printf("do_poser %d using test input\n", __LINE__);
#endif // READ_INPUT


#ifdef SAVE_OUTPUT
// write output frames to test_output

#endif // SAVE_OUTPUT

}






int main(int argc, char *argv[])
{
    init_gui();

#ifdef LOAD_COORDS
    do_debug_file();
#else // LOAD_COORDS

    do_poser();
#endif // !LOAD_COORDS
    return 0;
}






