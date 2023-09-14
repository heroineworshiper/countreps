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


// This creates an engine from an ONNX model.
// The ONNX model should previously have been made by convert2onnx.py in the
// https://github.com/spacewalk01/tensorrt-openpose package.

// python3 convert2onnx.py -i ../trt_pose/tasks/human_pose/resnet18_baseline_att_224x224_A_epoch_249.pth -o trt_pose.onnx

// make -f Makefile.trt
// ./trt_engine ../tensorrt-openpose/trt_pose.onnx trt_pose_fp16.engine
// ./trt_engine body25_fixed.onnx body25_fixed_fp16.engine

#include <cuda.h>
#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include "NvInferImpl.h"
#include <NvOnnxParser.h>
#include <NvInferLegacyDims.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>



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


int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("Usage: %s <onnx model> <engine filemame>\n", argv[0]);
        exit(0);
    }
    
    cudaSetDevice(0);
    Logger gLogger;
    char *model = argv[1];
    char *dst = argv[2];
	auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger));
	const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
	nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicitBatch);

	auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger));
    printf("loading model %s\n", model);

    if(!parser->parseFromFile(model, 2))
    {
        printf("parsing %s failed\n", model);
        exit(1);
    }

	builder->setMaxBatchSize(1);
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    config->setMaxWorkspaceSize(2 << 30);
    config->setFlag(nvinfer1::BuilderFlag::kFP16);

    printf("optimizing model\n");
    auto mem = std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
    printf("saving to %s\n", dst);
    FILE *fd = fopen(dst, "w");
    if(!fd)
    {
        printf("opening %s failed\n", dst);
        exit(1);
    }
    fwrite(mem->data(), 1, mem->size(), fd);
    fclose(fd);

    return 0;
}







