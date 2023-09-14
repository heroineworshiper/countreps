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

#include "body25.h"
#include "poseParameters.hpp"
#include "resize.h"
#include "point.hpp"
#include "resizeAndMergeBase.hpp"
#include "nmsBase.hpp"
#include "bodyPartConnectorBase.hpp"





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









std::size_t Engine::getSizeByDim(const nvinfer1::Dims& dims)
{
	std::size_t size = 1;
	for (std::size_t i = 0; i < dims.nbDims; ++i)
	{
		size *= dims.d[i];
	}
	return size;
}

double Engine::resizeGetScaleFactor(int w1, int h1, int w2, int h2)
{
    const auto ratioWidth = (double)(w2 - 1) / (double)(w1 - 1);
    const auto ratioHeight = (double)(h2 - 1) / (double)(h1 - 1);
    return MIN(ratioWidth, ratioHeight);
}


template<typename T>
int Engine::positiveIntRound(const T a)
{
    return int(a+0.5f);
}


void Engine::load(const char *path, 
    int cam_w, 
    int cam_h,
    int hdmi_w,
    int hdmi_h)
{
    this->hdmi_w = hdmi_w;
    this->hdmi_h = hdmi_h;

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

	cudaMalloc(&hdmiCUDA, hdmi_w * hdmi_h * 2); 
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


void Engine::process(int yuv_format, uint8_t *src)
{
    mPoseKeypoints.setTo(0);
    if(yuv_format != YUV_NONE)
    {
        int bytes = 0;
        if(yuv_format == YUV_PACKED) bytes = hdmi_w * hdmi_h * 2;
        if(yuv_format == YUV_PLANAR) bytes = hdmi_w * hdmi_h * 3 / 2;
// transfer input to GPU
	    cudaMemcpy(hdmiCUDA,  // dst
            src, // src
            bytes, 
            cudaMemcpyHostToDevice);
// convert to network size & RGB
        if(yuv_format == YUV_PACKED)
	        resizeAndNorm_yuv(hdmiCUDA,   // src
                (float*)cudaBuffers[0],  // dst
                hdmi_w, // srcW
                hdmi_h, // srcH
                inputW, // dstW
                inputH, // dstH
                cudaStream);
        else
        if(yuv_format == YUV_PLANAR)
            resizeAndNorm_yuv_planar(hdmiCUDA,   // src
                (float*)cudaBuffers[0],  // dst
                hdmi_w, // srcW
                hdmi_h, // srcH
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

    }

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





