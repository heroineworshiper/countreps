#include "Openpose.h"


#include <math.h>

#define CIRCLE_W 16
#define CIRCLE_H 16
uint8_t *circle_mask = 0;
#define BRUSH_W 4
#define BRUSH_H 4

extern int decoded_w;
extern int decoded_h;
extern unsigned char *input_image;
int r;
int g;
int b;

void init_circle_mask()
{
    circle_mask = new uint8_t[CIRCLE_W * CIRCLE_H];
    memset(circle_mask, 0, CIRCLE_W * CIRCLE_H);
    for(int i = 0; i < CIRCLE_H / 2; i++)
    {
        int r = (int)sqrt(CIRCLE_W * CIRCLE_W / 4 - i * i);
        uint8_t *row1 = circle_mask + CIRCLE_W * (CIRCLE_H / 2 + i);
        uint8_t *row2 = circle_mask + CIRCLE_W * (CIRCLE_H / 2 - i - 1);
        for(int j = CIRCLE_W / 2 - r; j < CIRCLE_W / 2 + r; j++)
        {
            row1[j] = 1;
            row2[j] = 1;
        }
    }
    
    
//     for(int i = 0; i < CIRCLE_H; i++)
//     {
//         for(int j = 0; j < CIRCLE_W; j++)
//             printf("%d", circle_mask[i * CIRCLE_W + j]);
//         printf("\n");
//     }
}

void draw_point(int x, 
    int y)
{
    for(int i = 0; i < CIRCLE_H; i++)
    {
        int row = y - CIRCLE_H / 2 + i;
        uint8_t *mask_row = &circle_mask[i * CIRCLE_W];
        if(row >= 0 && row < decoded_h)
        {
            int col0 = x - CIRCLE_W / 2;
            uint8_t *row_ptr = input_image + decoded_w * 3 * row;
            for(int j = 0; j < CIRCLE_W; j++)
            {
                int col = col0 + j;
                if(col >= 0 && col < decoded_w && mask_row[j])
                {
                    int offset = col * 3;
                    row_ptr[offset] = r;
                    row_ptr[offset + 1] = g;
                    row_ptr[offset + 2] = b;
                }
            }
        }
    }
}

void draw_brush(int x,
    int y)
{
    for(int i = 0; i < BRUSH_H; i++)
    {
        int row = y - BRUSH_H / 2 + i;
        if(row >= 0 && row < decoded_h)
        {
            int col0 = x - BRUSH_W / 2;
            uint8_t *row_ptr = input_image + decoded_w * 3 * row;
            for(int j = 0; j < BRUSH_W; j++)
            {
                int col = col0 + j;
                if(col >= 0 && col < decoded_w)
                {
                    int offset = col * 3;
                    row_ptr[offset] = r;
                    row_ptr[offset + 1] = g;
                    row_ptr[offset + 2] = b;
                }
            }
        }
    }
}

void draw_line(int x1, 
    int y1, 
    int x2, 
    int y2)
{
	int x_diff = labs(x2 - x1);
	int y_diff = labs(y2 - y1);
    if(!x_diff && !y_diff)
    {
        draw_brush(x1, y1);
    }
    else
    if(x_diff > y_diff)
    {
        if(x2 < x1)
        {
            int temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }

		int n = y2 - y1;
		int d = x2 - x1;
		for(int i = x1; i <= x2; i++)
		{
			int y = y1 + (int64_t)(i - x1) * (int64_t)n / (int64_t)d;
            draw_brush(i, y);
		}
    }
    else
    {
        if(y2 < y1)
        {
            int temp = y1;
            y1 = y2;
            y2 = temp;
            temp = x1;
            x1 = x2;
            x2 = temp;
        }

		int n = x2 - x1;
		int d = y2 - y1;
		for(int i = y1; i <= y2; i++)
		{
			int x = x1 + (int64_t)(i - y1) * (int64_t)n / (int64_t)d;
			draw_brush(x, i);
		}
    }
}


void Openpose::detect(const float *cmap_ptr, 
    const float *paf_ptr, 
    uint8_t *frame,
    int decoded_w,
    int decoded_h)
{
	/*
	 Input arguments:
		cmap: feature maps of joints
		paf: connections between joints
		frame: image data

	 output arguments:
		object_counts_ptr[0];			// N
		objects_ptr[0];					// NxMxC
		refined_peaks_ptr[0];			// NxCxMx2
	*/

	// ****** DETECT SKELETON ***** //

	// 1. Find peaks (NMS)

	size_t peak_size = N * C * M * 2;	// NxCxMx2
	int *peaks = new int[peak_size];

	size_t peak_count_size = N * C; // NxC
	int *peak_counts = new int[peak_count_size];

	trt_pose::parse::find_peaks_out_nchw(peak_counts, peaks, cmap_ptr, N, C, H, W, M, cmap_threshold, cmap_window);

	// 2. Refine peaks

	float *refined_peaks = new float[peak_size]; // NxCxMx2

	for (int i = 0; i < peak_size; i++) refined_peaks[0] = 0;

	trt_pose::parse::refine_peaks_out_nchw(refined_peaks, 
        peak_counts, 
        peaks, 
        cmap_ptr, 
        N, 
        C, 
        H, 
        W, 
        M, 
        cmap_window);

printf("Openpose::detect %d peak_size=%d peak_count_size=%d\n", 
__LINE__, 
peak_size,
peak_count_size);
	// 3. Score paf 
 
	int K = 21;

	size_t score_graph_size = N * K * M * M;	// NxKxMxM
	float *score_graph = new float[score_graph_size];

	trt_pose::parse::paf_score_graph_out_nkhw(score_graph, 
        topology, 
        paf_ptr, 
        peak_counts, 
        refined_peaks,
		N, 
        K, 
        C, 
        H, 
        W, 
        M, 
        line_integral_samples);

	// 4. Assignment algorithm

	int *connections = new int[N * K * 2 * M];
	int connection_size = N * K * 2 * M;
	for (int i = 0; i < connection_size; i++) connections[i] = -1.0;

	void *workspace = (void *)malloc(trt_pose::parse::assignment_out_workspace(M));

	trt_pose::parse::assignment_out_nk(connections, 
        score_graph, 
        topology, 
        peak_counts, 
        N, 
        C, 
        K, 
        M, 
        link_threshold, 
        workspace);

	// 5. Merging

	int *objects = new int[N * max_num_objects * C];
	for (int i = 0; i < N * max_num_objects * C; i++) objects[i] = -1;

	int *object_counts = new int[N];
	object_counts[0] = 0;	// batchSize=1		

	void *merge_workspace = malloc(trt_pose::parse::connect_parts_out_workspace(C, M));

	trt_pose::parse::connect_parts_out_batch(object_counts, 
        objects, 
        connections, 
        topology, 
        peak_counts, 
        N, 
        K, 
        C, 
        M, 
        max_num_objects, 
        merge_workspace);

	// ****** DRAWING SKELETON ***** //

    if(!circle_mask)
    {
        init_circle_mask();
    }

//printf("Openpose::detect %d N=%d C=%d K=%d max_num_objects=%d %d\n", __LINE__, N, C, K, max_num_objects, object_counts[0]);
	for (int i = 0; i < object_counts[0]; i++) 
    {

		int *obj = &objects[C * i];

		for (int j = 0; j < C; j++) {

			int k = (int)obj[j];
			if (k >= 0) {
				float *peak = &refined_peaks[j * M * 2];
				int x = (int)(peak[k * 2 + 1] * decoded_w);
				int y = (int)(peak[k * 2] * decoded_h);
                r = 0;
                g = 0xff;
                b = 0;
                draw_point(x, y);
				peak = NULL;
			}
		}

		for (int k = 0; k < K; k++) 
        {
			int c_a = topology[k * 4 + 2];
			int c_b = topology[k * 4 + 3];

			if (obj[c_a] >= 0 && obj[c_b] >= 0) {
				float *peak0 = &refined_peaks[c_a * M * 2];
				float *peak1 = &refined_peaks[c_b * M * 2];

				int x0 = (int)(peak0[(int)obj[c_a] * 2 + 1] * decoded_w);
				int y0 = (int)(peak0[(int)obj[c_a] * 2] * decoded_h);
				int x1 = (int)(peak1[(int)obj[c_b] * 2 + 1] * decoded_w);
				int y1 = (int)(peak1[(int)obj[c_b] * 2] * decoded_h);
                r = 0;
                g = 0xff;
                b = 0;
                draw_line(x0, y0, x1, y1);
				//line(frame, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2, 1);

// 				if ((c_a == 5 && c_b == 7) || (c_a == 7 && c_b == 9))
// 				{
// 					arrowedLine(frame, 
//                         cv::Point(x0, y0), 
//                         cv::Point(x1, y1), 
//                         cv::Scalar(0, 0, 255), 
//                         2, 
//                         1); // red				
// 				}
// 				else if ((c_a == 6 && c_b == 8) || (c_a == 8 && c_b == 10))
// 				{
// 					arrowedLine(frame, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255, 0, 0), 2, 1); // blue
// 				}
// 				else 
//                 {
// 					if ((c_b == 11 && c_a == 13) || (c_b == 13 && c_a == 15) || (c_b == 12 && c_a == 14) || (c_b == 14 && c_a == 16))
// 					{
// 						line(frame, cv::Point(x1, y1), cv::Point(x0, y0), cv::Scalar(0, 255, 0), 2, 1); // green
// 					}
// 					else
// 					{
// 						line(frame, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2, 1); // green
// 					}
// 				}

				//cv::putText(frame, std::to_string(c_a), cv::Point(x0, y0), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, false);
				//cv::putText(frame, std::to_string(c_b), cv::Point(x1, y1), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, false);
			}
		}


		obj = NULL;
	}

	delete[] peaks;
	peaks = NULL;
	delete[] peak_counts;
	peak_counts = NULL;

	delete[] refined_peaks;
	refined_peaks = NULL;

	paf_ptr = NULL;
	delete[] score_graph;
	score_graph = NULL;

	delete[] connections;
	connections = NULL;

	delete[] objects;
	objects = NULL;

	delete[] object_counts;
	object_counts = NULL;

	std::free(workspace);
	std::free(merge_workspace);

}
