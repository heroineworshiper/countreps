// resize & norm for trt_pose

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>

__forceinline__ __device__ float3 get(uchar3* src, int x, int y, int w, int h) 
{
	if (x < 0 || x >= w || y < 0 || y >= h) return make_float3(0.5, 0.5, 0.5);
	uchar3 temp = src[y*w + x];
// trt_pose range
	return make_float3(float(temp.x) / 255., float(temp.y) / 255., float(temp.z) / 255.);
}

__global__ void resizeNormKernel_openpose(uchar3* src, 
    float *dst, 
    int dstW, 
    int dstH, 
    int srcW, 
    int srcH,
	float scaleX, 
    float scaleY, 
    float shiftX, 
    float shiftY)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const int x = idx % dstW;
	int y = idx / dstW;
	if (x >= dstW || y >= dstH)
		return;
	float w = (x - shiftX + 0.5) * scaleX - 0.5;
	float h = (y - shiftY + 0.5) * scaleY - 0.5;
	int h_low = (int)h;
	int w_low = (int)w;
	int h_high = h_low + 1;
	int w_high = w_low + 1;
	float lh = h - h_low;
	float lw = w - w_low;
	float hh = 1.0 - lh, hw = 1.0 - lw;
	float w1 = hh * hw, w2 = hh * lw, w3 = lh * hw, w4 = lh * lw;
	float3 v1 = get(src, w_low, h_low, srcW, srcH);
	float3 v2 = get(src, w_high, h_low, srcW, srcH);
	float3 v3 = get(src, w_low, h_high, srcW, srcH);
	float3 v4 = get(src, w_high, h_high, srcW, srcH);
	int stride = dstW * dstH;
//	dst[y*dstW + x] = w1 * v1.x + w2 * v2.x + w3 * v3.x + w4 * v4.x;
//	dst[stride + y * dstW + x] = w1 * v1.y + w2 * v2.y + w3 * v3.y + w4 * v4.y;
//	dst[stride * 2 + y * dstW + x] = w1 * v1.z + w2 * v2.z + w3 * v3.z + w4 * v4.z;



	float r = w1 * v1.x + w2 * v2.x + w3 * v3.x + w4 * v4.x;
	float g = w1 * v1.y + w2 * v2.y + w3 * v3.y + w4 * v4.y;
	float b = w1 * v1.z + w2 * v2.z + w3 * v3.z + w4 * v4.z;


// extra step which improves performance in trt_pose
// image.sub_(mean[:, None, None]).div_(std[:, None, None])
//      r -= 0.485;
//      g -= 0.456;
//      b -= 0.406;
//      r /= 0.229;
//      g /= 0.224;
//      b /= 0.225;


    dst[y*dstW + x] = r;
    dst[stride + y * dstW + x] = g;
    dst[stride * 2 + y * dstW + x] = b;
}

// Decompression coefficients straight out of jpeglib
#define V_TO_R    1.40200
#define V_TO_G    -0.71414
#define U_TO_G    -0.34414
#define U_TO_B    1.77200


__device__ float3 getYUV(uchar1* src, 
    int x, 
    int y, 
    int w, 
    int h) 
{
//	if (x < 0 || x >= w || y < 0 || y >= h) return make_float3(0.0, 0.0, 0.0);
    float3 yuv;
    int offset_uv = y * w * 2 + x / 2 * 4;
	yuv.x = float(src[y * w * 2 + x * 2].x) / 255;
    yuv.y = float(src[offset_uv + 1].x) / 255 - 0.5;
    yuv.z = float(src[offset_uv + 3].x) / 255 - 0.5;

    float3 rgb;
	rgb.x = yuv.x + V_TO_R * yuv.z;
	rgb.y = yuv.x + U_TO_G * yuv.y + V_TO_G * yuv.z;
	rgb.z = yuv.x + U_TO_B * yuv.y;
    return rgb;
}

__global__ void resizeNormKernel_landscape(uchar1* src, 
    float *dst, 
    int dstW, 
    int dstH, 
    int srcW, 
    int srcH,
	float scaleX, 
    float scaleY)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const int x = idx % dstW;
	int y = idx / dstW;
	if (x >= dstW || y >= dstH)
		return;
	float w = (x + 0.5) * scaleX - 0.5;
	float h = (y + 0.5) * scaleY - 0.5;
	int h_low = (int)h;
	int w_low = (int)w;
	int h_high = h_low + 1;
	int w_high = w_low + 1;
	float lh = h - h_low;
	float lw = w - w_low;
	float hh = 1.0 - lh, hw = 1.0 - lw;
	float w1 = hh * hw, w2 = hh * lw, w3 = lh * hw, w4 = lh * lw;
	float3 v1 = getYUV(src, w_low, h_low, srcW, srcH);
	float3 v2 = getYUV(src, w_high, h_low, srcW, srcH);
	float3 v3 = getYUV(src, w_low, h_high, srcW, srcH);
	float3 v4 = getYUV(src, w_high, h_high, srcW, srcH);
	int stride = dstW * dstH;

	float r = w1 * v1.x + w2 * v2.x + w3 * v3.x + w4 * v4.x;
	float g = w1 * v1.y + w2 * v2.y + w3 * v3.y + w4 * v4.y;
	float b = w1 * v1.z + w2 * v2.z + w3 * v3.z + w4 * v4.z;


    dst[y*dstW + x] = r;
    dst[stride + y * dstW + x] = g;
    dst[stride * 2 + y * dstW + x] = b;
}

__global__ void resizeNormKernel_portrait(uchar1* src, 
    float *dst, 
    int dstW, 
    int dstH, 
    int srcW, 
    int srcH,
	float scaleX, 
    float scaleY, 
    float shiftX, 
    float shiftY)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const int x = idx % dstW;
	int y = idx / dstW;
	if (x >= dstW || y >= dstH)
		return;
// rotate & scale
	float w = (-y + 0.5) * scaleX + shiftX - 0.5;
	float h = (x + 0.5) * scaleY - shiftY - 0.5;
	int h_low = (int)h;
	int w_low = (int)w;
	int h_high = h_low + 1;
	int w_high = w_low + 1;
	float lh = h - h_low;
	float lw = w - w_low;
	float hh = 1.0 - lh, hw = 1.0 - lw;
	float w1 = hh * hw, w2 = hh * lw, w3 = lh * hw, w4 = lh * lw;
	float3 v1 = getYUV(src, w_low, h_low, srcW, srcH);
	float3 v2 = getYUV(src, w_high, h_low, srcW, srcH);
	float3 v3 = getYUV(src, w_low, h_high, srcW, srcH);
	float3 v4 = getYUV(src, w_high, h_high, srcW, srcH);
	int stride = dstW * dstH;

	float r = w1 * v1.x + w2 * v2.x + w3 * v3.x + w4 * v4.x;
	float g = w1 * v1.y + w2 * v2.y + w3 * v3.y + w4 * v4.y;
	float b = w1 * v1.z + w2 * v2.z + w3 * v3.z + w4 * v4.z;


    dst[y*dstW + x] = r;
    dst[stride + y * dstW + x] = g;
    dst[stride * 2 + y * dstW + x] = b;
}

int resizeAndNorm_yuv(void *src, 
    float *dst, 
    int src_w, 
    int src_h, 
    int dst_w, 
    int dst_h, 
    cudaStream_t stream) 
{
	float scaleX;
	float scaleY;
	float shiftX = 0.f;
    float shiftY = 0.f;

	const int n = dst_w * dst_h;
	int blockSize = 512;
	const int gridSize = (n + blockSize - 1) / blockSize;

    if(dst_w > dst_h)
    {
// landscape
    	scaleX = src_w * 1.0f / dst_w;
	    scaleY = src_h * 1.0f / dst_h;
        resizeNormKernel_landscape << <gridSize, blockSize, 0, stream >> > ((uchar1*)(src), dst, dst_w, dst_h, src_w, src_h, scaleX, scaleY);
    }
    else
    {
// portrait
// camera cropping for 3:2
        float src_w2 = src_w * 540 / 640;
        scaleX = src_w2 * 1.0f / dst_h;
        scaleY = src_h * 1.0f / dst_w;
        shiftX = (src_w2 - src_w) / 2;
        resizeNormKernel_portrait << <gridSize, blockSize, 0, stream >> > ((uchar1*)(src), dst, dst_w, dst_h, src_w, src_h, scaleX, scaleY, shiftX, shiftY);
// printf("resizeAndNorm_yuv %d src=%dx%d dst=%dx%d scaleX=%f scaleY=%f shiftX=%f\n",
// __LINE__,
// src_w,
// src_h,
// dst_w,
// dst_h,
// scaleX,
// scaleY,
// shiftX);
    }


//printf("resizeAndNorm %d %f %f %f %f\n", __LINE__, scaleX, scaleY, shiftX, shiftY);
	return 0;
}








