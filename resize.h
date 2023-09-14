#pragma once

#ifndef TENSORRT_YOLOV4_RESIZE_H
#define TENSORRT_YOLOV4_RESIZE_H
typedef unsigned char uchar;
int resizeAndNorm(void * p, float *d, int w, int h, int in_w, int in_h, cudaStream_t stream);
int resizeAndNorm_yuv(void *src, 
    float *dst, 
    int src_w, 
    int src_h, 
    int dst_w, 
    int dst_h, 
    cudaStream_t stream);
int resizeAndNorm_yuv_planar(void *src, 
    float *dst, 
    int src_w, 
    int src_h, 
    int dst_w, 
    int dst_h, 
    cudaStream_t stream);
#endif //TENSORRT_YOLOV4_RESIZE_H
