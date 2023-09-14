/*
 * Body25 wrapper
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





#ifndef BODY25_H
#define BODY25_H



#include "array.hpp"
#include <stdint.h>
#include "NvInfer.h"
#include <vector>




#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define TO_MS(x) ((x).tv_sec * 1000 + (x).tv_nsec / 1000000)
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define BODY_PARTS 25
#define YUV_NONE 0
#define YUV_PACKED 1
#define YUV_PLANAR 2




class Engine
{
public:
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    std::vector<nvinfer1::Dims> inputDims;
    std::vector<nvinfer1::Dims> outputDims;
    int batchSize;
    int numChannels;
    int hdmi_w;
    int hdmi_h;
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

    std::size_t getSizeByDim(const nvinfer1::Dims& dims);
    double resizeGetScaleFactor(int w1, int h1, int w2, int h2);
    template<typename T> int positiveIntRound(const T a);
    void load(const char *path, 
        int cam_w, 
        int cam_h,
        int hdmi_w,
        int hdmi_h);
    void process(int yuv_format, uint8_t *src);
};

#endif

