

// generate the speech with https://www.voicery.com/
// use a chrome extension to capture the audio
// select Katie, enable ssml
// paste blocks of text until the counter fills up

// 0<break/> 1<break/> 2 <break/>3 <break/>4 <break/>5 <break/>6 <break/>7 <break/>8 
// <break/>9 <break/>10 <break/>11 <break/>12 <break/>13 <break/>14 <break/>15 

// <break/>16 <break/>17 <break/>18 <break/>19 <break/>20 <break/>21 <break/>22 
// <break/>23 <break/>24 <break/>25 <break/>26 <break/>27 <break/>28 <break/>29 
// <break/>30 <break/>31 <break/>32 <break/>33

// begin situps<break/>begin pushups<break/>begin hip flexes<break/>begin squats<break/>
// welcome to rep counter
// the program has finished


#include <ft2build.h>
#include FT_FREETYPE_H
#include "condition.h"
#include "guicast.h"
#include "keys.h"
#include "mutex.h"
#include "gui.h"
#include "countreps.h"

//#define W 1920
//#define H 1040
#define W 1366
#define H 768
// where to draw the vijeo
#define VIDEO_W (H * w / h)
//#define VIDEO_W (H * 16 / 9)
//#define VIDEO_W H
#define VIDEO_H H
#define VIDEO_X ((W - VIDEO_W) / 2)
#define TEXT_X VIDEO_W
//#define FONT "impact.ttf"
#define FONT "arial.ttf"
//#define BIGTEXT 512
//#define LITTLETEXT 256
#define BIGTEXT 256
#define LITTLETEXT 128
#define FPS_SIZE 50

// from include/openpose/pose/poseParametersRender.hpp: POSE_BODY_25_COLORS_RENDER_GPU
uint32_t colors[] =
{
    0x80ff0055, // NOSE
    0x80ff0000, // NECK
    0x80ff5500, // SHOULDER2
    0x80ffaa00, // ELBOW2 orange
    0x80ffff00, // WRIST2
    0x80aaff00, // SHOULDER1 light green
    0x8055ff00, // ELBOW1
    0x8000ff00, // WRIST1
    0x80ff0000, // BUTT
    0x8000ff55, // HIP2
    0x8000ffaa,  // KNEE2
    0x8000ffff, // ANKLE2
    0x8000aaff, // HIP1
    0x800055ff, // KNEE1
    0x800000ff, // ANKLE1
    0x80ff00aa, // REYE
    0x80aa00ff, // LEYE
    0x80ff00ff, // LEAR
    0x805500ff, // REAR
    0x800000ff, 
    0x800000ff, 
    0x800000ff, 
    0x8000ffff, 
    0x8000ffff, 
    0x8000ffff, 
};
#define TOTAL_COLORS (sizeof(colors) / sizeof(int))

const char *exercise_to_text[] =
{
    "SIT\nUP",
    "PUSH\nUP",
    "HIP\nFLEX",
    "SQU\nAT",
};

int prev_reps = -1;
int prev_exercise = -1;

  
class SoundThread : public Thread
{
public:
    SoundThread() : Thread(1)
    {
    }
    
    void run()
    {
        while(1)
        {
            blurb_wait.lock();
            
            blurb_lock.lock();
            if(blurbs.size() > 0)
            {
                char *blurb = blurbs.get(0);
                blurbs.remove_number(0);
                blurb_lock.unlock();
                
                if(!blurb)
                {
                    break;
                }
                
                char string[BCTEXTLEN];
                sprintf(string, "aplay %s", blurb);
                int _ = system(string);
                
                free(blurb);
            }
            else
            {
                blurb_lock.unlock();
            }
        }
    }
    
    void play_sound(const char *path)
    {
        char *text = 0;
        
        if(path)
        {
            text = strdup(path);
        }
        blurb_lock.lock();
        blurbs.append(text);
        blurb_lock.unlock();
        blurb_wait.unlock();
    }
    
    
    ArrayList<char*> blurbs;
    Condition blurb_wait;
    Mutex blurb_lock;
};

class GUI : public BC_Window
{
public:
    GUI() : BC_Window("Countreps",
        0, // x
		0, // y
		W, // w
		H, // h
		-1, // minw
		-1, // minh
		0, // allow_resize
		0, // private_color
		1, // hide
        WHITE) // bg_color
    {
    };

	int close_event()
	{
		set_done(0);
		return 1;
	};
    
    int keypress_event()
    {
        //printf("GUI::keypress_event %d %c\n", __LINE__, get_keypress());
        switch(get_keypress())
        {
            case ESC:
            case 'q':
                set_done(0);
                return 1;
                break;
        }
        return 0;
    }
    
};

GUI *gui;
class GUIThread : public Thread
{
public:
    GUIThread() : Thread()
    {
    }

    void run()
    {
        gui->run_window();
        exit(0);
    }
};


GUIThread thread;
SoundThread sound;
BC_Bitmap *bitmap = 0;
FT_Library freetype_library = 0;
FT_Face freetype_face = 0;


int load_freetype_face(FT_Library &freetype_library,
	FT_Face &freetype_face,
	const char *path)
{
	if(freetype_face) FT_Done_Face(freetype_face);
	freetype_face = 0;

// Use freetype's internal function for loading font
	if(FT_New_Face(freetype_library, 
		path, 
		0,
		&freetype_face))
	{
		printf("load_freetype_face %s failed.\n", path);
		freetype_face = 0;
		return 1;
	} 
    else
	{
		return 0;
	}

}


void say_intro()
{
    sound.play_sound("welcome.wav");
}

void say_reps(int number)
{
    const char *paths[] = 
    {
        "0.wav",
        "1.wav",
        "2.wav",
        "3.wav",
        "4.wav",
        "5.wav",
        "6.wav",
        "7.wav",
        "8.wav",
        "9.wav",
        "10.wav",
        "11.wav",
        "12.wav",
        "13.wav",
        "14.wav",
        "15.wav",
        "16.wav",
        "17.wav",
        "18.wav",
        "19.wav",
        "20.wav",
        "21.wav",
        "22.wav",
        "23.wav",
        "24.wav",
        "25.wav",
        "26.wav",
        "27.wav",
        "28.wav",
        "29.wav",
        "30.wav",
        "31.wav",
        "32.wav",
        "33.wav"
    };
    
    sound.play_sound(paths[number]);
}

void say_exercise(int exercise)
{
    const char *paths[] = 
    {
        "situps.wav",
        "pushups.wav",
        "hips.wav",
        "squats.wav"
    };
    sound.play_sound(paths[exercise]);
}

void init_gui()
{
    sound.start();
    
    FT_Init_FreeType(&freetype_library);
    load_freetype_face(freetype_library,
		freetype_face,
		FONT);

    
    gui = new GUI();
// don't use a back buffer
//    gui->start_video();
    bitmap = new BC_Bitmap(gui, 
	    W,
	    H,
	    BC_BGR8888,
	    1); // use_shm
    
    gui->show_window();
    say_intro();

    thread.start();
}




void draw_text(int x, int y, int size, const char *text)
{
    int len = strlen(text);
    unsigned char **dst_rows = bitmap->get_row_pointers();
// The text color
    int r = 0;
    int g = 0xff;
    int b = 0;
    int out_x = x;
    int out_y = y;

    FT_Set_Pixel_Sizes(freetype_face, size, 0);

    for(int i = 0; i < len; i++)
    {
        if(text[i] == '\n')
        {
            out_y += size;
            out_x = x;
        }
        else
        {
            if(!FT_Load_Char(freetype_face, text[i], FT_LOAD_RENDER))
		    {
                int char_ascent = freetype_face->glyph->bitmap_top;
                int in_h = freetype_face->glyph->bitmap.rows;
                int in_w = freetype_face->glyph->bitmap.width;
                int advance = freetype_face->glyph->advance.x >> 6;
                for(int in_y = 0, out_y2 = out_y - char_ascent; 
                    in_y < in_h; 
                    in_y++, out_y2++)
                {
                    if(out_y2 >= 0 && out_y2 < H)
                    {
                        int out_x2 = out_x;
                        unsigned char *out_row = dst_rows[out_y2] +
					        out_x * 4;
                        unsigned char *in_row = freetype_face->glyph->bitmap.buffer + 
					        freetype_face->glyph->bitmap.pitch * in_y;

                        for(int in_x = 0; in_x < in_w; in_x++)
                        {
                            if(out_x2 >= 0 && out_x2 < W)
                            {
                                int v = *in_row;
                                *out_row++ = (v * b + 
						            (0xff - v) * *out_row) / 0xff;
					            *out_row++ = (v * g +
						            (0xff - v) * *out_row) / 0xff;
					            *out_row++ = (v * r +
						            (0xff - v) * *out_row) / 0xff;
                                out_row++;
                            }
                            else
                            {
                                out_row += 4;
                            }

                            in_row++;
                            out_x2++;
                        }
                    }
                }

                out_x += advance;
            }
        }
    }
    
}

void update_gui(unsigned char *src, 
    int w, 
    int h, 
    int reps, 
    int exercise,
    float fps,
    int bgr)
{
    if (reps != prev_reps || exercise != prev_exercise) {

        if (reps != prev_reps) 
        {
            say_reps(reps);
        }
        prev_reps = reps;

        if (exercise != prev_exercise) 
        {
            say_exercise(exercise);
        }
        prev_exercise = exercise;
    }




    unsigned char **dst_rows = bitmap->get_row_pointers();

    memset(dst_rows[0], 0xff, bitmap->get_bytes_per_line() * bitmap->get_h());

// draw the video
    int nearest_x[VIDEO_W];
    int nearest_y[VIDEO_H];
    for(int i = 0; i < VIDEO_W; i++)
    {
        nearest_x[i] = i * w / VIDEO_W;
    }
    for(int i = 0; i < VIDEO_H; i++)
    {
        nearest_y[i] = i * h / VIDEO_H;
    }
    
    for(int i = 0; i < VIDEO_H; i++)
    {
        unsigned char *src_row = src + nearest_y[i] * w * 3;
        unsigned char *dst_row = dst_rows[i] + VIDEO_X * 4;


        if(bgr)
        {
            for(int j = 0; j < VIDEO_W; j++)
            {
                int src_x = nearest_x[j];
    // SRC=RGB DST=BGR
                *dst_row++ = src_row[src_x * 3 + 2];
                *dst_row++ = src_row[src_x * 3 + 1];
                *dst_row++ = src_row[src_x * 3 + 0];
                dst_row++;
            }
        }
        else
        {
            for(int j = 0; j < VIDEO_W; j++)
            {
                int src_x = nearest_x[j];
    // SRC=RGB DST=RGB
                *dst_row++ = src_row[src_x * 3 + 0];
                *dst_row++ = src_row[src_x * 3 + 1];
                *dst_row++ = src_row[src_x * 3 + 2];
                dst_row++;
            }
        }
    }    

// draw the status
    char string[BCTEXTLEN];

    if(fps > 0)
    {
        sprintf(string, "%.2f FPS", fps);
        draw_text(25, 50, FPS_SIZE, string);
    }
//     sprintf(string, "%d", reps);
//     draw_text(TEXT_X, H / 2, BIGTEXT, string);
//     draw_text(TEXT_X, H * 3 / 4, LITTLETEXT, exercise_to_text[exercise]);

    

    gui->lock_window();
    gui->draw_bitmap(bitmap, 
	    1,
	    0, 
	    0,
	    W,
	    H,
	    0,
	    0,
	    W,
	    H);

    gui->unlock_window();
}


void join_body_parts(int bodyPart1, 
    int bodyPart2, 
    int *xy_array, 
    int decoded_w, 
    int decoded_h)
{
    int x1 = xy_array[bodyPart1 * 2 + 0] * W / decoded_w;
    int y1 = xy_array[bodyPart1 * 2 + 1] * H / decoded_h;
    int x2 = xy_array[bodyPart2 * 2 + 0] * W / decoded_w;
    int y2 = xy_array[bodyPart2 * 2 + 1] * H / decoded_h;
    if((x1 > 0 || y1 > 0) && (x2 > 0 || y2 > 0))
    {
        gui->set_color(colors[bodyPart1] & 0xffffff);
        gui->set_line_width(5);
        gui->draw_line(x1, y1, x2, y2);
        gui->set_line_width(1);
    }
}


void draw_body(int *xy_array, int decoded_w, int decoded_h)
{
    gui->lock_window();

// draw the body parts
    for(int j = 0; j < TOTAL_COLORS; j++)
    {
        int x = xy_array[j * 2] * W / decoded_w;
        int y = xy_array[j * 2 + 1] * H / decoded_h;
//printf("draw_body %d: %d %d\n", __LINE__, x, y);
        if(x > 0 || y > 0)
        {
#define PART_SIZE 20
            gui->set_color(colors[j] & 0xffffff);
            gui->draw_disc(x - PART_SIZE / 2, y - PART_SIZE / 2, PART_SIZE, PART_SIZE);
        }
    }

    join_body_parts(MODEL_SHOULDER1, MODEL_NECK, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_SHOULDER2, MODEL_NECK, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_ELBOW2, MODEL_SHOULDER2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_ELBOW1, MODEL_SHOULDER1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_WRIST2, MODEL_ELBOW2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_WRIST1, MODEL_ELBOW1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_NECK, MODEL_BUTT, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_HIP1, MODEL_BUTT, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_HIP2, MODEL_BUTT, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_KNEE1, MODEL_HIP1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_KNEE2, MODEL_HIP2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_ANKLE1, MODEL_KNEE1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_ANKLE2, MODEL_KNEE2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_NOSE, MODEL_NECK, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_EYE2, MODEL_NOSE, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_EYE1, MODEL_NOSE, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_EAR1, MODEL_EYE1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_EAR2, MODEL_EYE2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_HEEL1, MODEL_ANKLE1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_HEEL2, MODEL_ANKLE2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_BIGTOE1, MODEL_ANKLE1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_BIGTOE2, MODEL_ANKLE2, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_SMALLTOE1, MODEL_BIGTOE1, xy_array, decoded_w, decoded_h);
    join_body_parts(MODEL_SMALLTOE2, MODEL_BIGTOE2, xy_array, decoded_w, decoded_h);


    gui->unlock_window();
}

void flash_gui()
{
    gui->lock_window();
    gui->flash();
    gui->unlock_window();
}

// called outside the GUIThread
void finish_gui()
{
    sound.play_sound("done.wav");
    sound.play_sound(0);
    sound.join();
    gui->lock_window();
    gui->set_done(0);
    gui->unlock_window();
}



