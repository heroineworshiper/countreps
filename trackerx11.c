/*
 * X11 GUI for the tracking camera
 * Copyright (C) 2019-2021 Adam Williams <broadcast at earthling dot net>
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

#include "keys.h"
#include "mutex.h"
#include "tracker.h"
#include <unistd.h>


// hard coded for testing on a bigger monitor
GUI *gui;



const char* lens_to_text(int lens)
{
    switch(lens)
    {
        case LENS_15:
            return "15-17MM";
            break;
        case LENS_28:
            return "28MM";
            break;
        case LENS_50:
            return "50MM";
            break;
        default:
            return "UNKNOWN";
    }
}

const char* landscape_to_text(int landscape)
{
    if(landscape)
    {
        return "LANDSCAPE";
    }
    else
    {
        return "PORTRAIT";
    }
}


GUI::GUI() : BC_Window("Tracker",
    0, // x
	0, // y
	WINDOW_W, // w
	WINDOW_H, // h
	-1, // minw
	-1, // minh
	0, // allow_resize
	0, // private_color
	1, // hide
    BLACK) // bg_color
{
    reserved_w = WINDOW_W * 3 / 4;
}

int GUI::close_event()
{
	set_done(0);
	return 1;
}
    
void GUI::print_values(int flash_it)
{
    char string[BCTEXTLEN];
    sprintf(string, 
        "PAN (a,d) = %d\n"
        "TILT (w,s) = %d\n"
        "PAN_SIGN (p) = %d\n"
        "TILT_SIGN (t) = %d\n"
        "LENS (l) = %s\n"
        "ROTATION (r) = %s", 
        (int)(pan - (MAX_PWM + MIN_PWM) / 2),
        (int)(tilt - (MAX_PWM + MIN_PWM) / 2),
        pan_sign,
        tilt_sign,
        lens_to_text(lens),
        landscape_to_text(landscape));
    int line_h = get_text_height(LARGEFONT, "0");
    int text_h = get_text_height(LARGEFONT, string);
    int text_w = get_text_width(LARGEFONT, "ROTATION (r) = LANDSCAPE");
    int text_x = MARGIN;
    int text_y = line_h + text_y2;
    clear_box(text_x, text_y - line_h, text_w, text_h);
    set_color(WHITE);
    draw_text(text_x, 
        text_y, 
        string);
    if(flash_it)
    {
        flash(text_x, text_y - line_h, text_w, text_h, 1);
    }
}
    
int GUI::keypress_event()
{
//        printf("GUI::keypress_event %d %c\n", __LINE__, get_keypress());
    int need_print_values = 0;
    int need_write_servos = 0;
    switch(get_keypress())
    {
        case ESC:
        case 'q':
// escape out of configuration
            if(current_operation == CONFIGURING)
            {
                ::save_defaults();
            }
            set_done(0);
            return 1;
            break;

        case ' ':
        case RETURN:
// advance operation
            if(current_operation == STARTUP)
            {
                current_operation = CONFIGURING;
                clear_box(0, 0, WINDOW_W, WINDOW_H, 0);
                int text_h = get_text_height(LARGEFONT, "q0");
                int y = text_h + MARGIN;
                int x = MARGIN;
                set_color(WHITE);
                char string[BCTEXTLEN];
                sprintf(string,
                    "Press keys to aim the mount.\n\n"
                    "PWM Values should be as\n"
                    "close to 0 as possible.\n"
//                         "a - left\n"
//                         "d - right\n"
//                         "w - up\n"
//                         "s - down\n"
//                         "t - invert tilt sign\n"
//                         "p - invert pan sign\n"
//                         "l - change lens\n"
//                         "r - rotate the camera\n"
                    "SPACE or ENTER to save defaults & \n"
                    "begin tracking\n"
                    "ESC to give up & go to a movie.");
                draw_text(x, 
                    y, 
                    string);
                text_y2 = y + get_text_height(LARGEFONT, string) + text_h;

                print_values(0);
                flash(1);

// write it a few times to defeat UART initialization glitches
                write_servos(1);
                usleep(100000);
                write_servos(1);
                usleep(100000);
                write_servos(1);
                usleep(100000);
                write_servos(1);
            }
            else
            if(current_operation == CONFIGURING)
            {
                start_pan = pan;
                start_tilt = tilt;
                ::save_defaults();
                current_operation = TRACKING;
                clear_box(0, 0, WINDOW_W, WINDOW_H, 0);
                flash(1);
            }
            break;

        case 'w':
            tilt += lenses[lens].tilt_step * tilt_sign;
            need_write_servos = 1;
            need_print_values = 1;
            break;

        case 's':
            tilt -= lenses[lens].tilt_step * tilt_sign;
            need_write_servos = 1;
            need_print_values = 1;
            break;

        case 'a':
            pan -= lenses[lens].pan_step * pan_sign;
            need_write_servos = 1;
            need_print_values = 1;
            break;

        case 'd':
            pan += lenses[lens].pan_step * pan_sign;
            need_write_servos = 1;
            need_print_values = 1;
            break;

        case 't':
            tilt_sign *= -1;
            need_print_values = 1;
            break;

        case 'p':
            pan_sign *= -1;
            need_print_values = 1;
            break;

        case 'r':
            landscape = !landscape;
            need_print_values = 1;
            need_clear_video = 1;
            break;

        case 'l':
            lens++;
            if(lens >= TOTAL_LENSES)
            {
                lens = 0;
            }
            need_print_values = 1;
            break;
    }

    if(need_print_values && current_operation == CONFIGURING)
    {
        print_values(1);
    }

    if(need_write_servos)
    {
        write_servos(1);
    }

// trap all of them
    return 1;
}


GUIThread::GUIThread() : Thread()
{
}

void GUIThread::run()
{
    gui->run_window();
    exit(0);
}


static GUIThread gui_thread;
static BC_Bitmap *gui_bitmap = 0;


void init_gui()
{
    gui = new GUI();
    gui->reposition_window(-gui->get_resources()->get_left_border(),
        -gui->get_resources()->get_top_border());
    gui_bitmap = new BC_Bitmap(gui, 
	    WINDOW_W,
	    WINDOW_H,
	    BC_BGR8888,
	    1); // use_shm
    gui->start_video();
    gui->show_window();

    gui_thread.start();
}



// draw a frame on the GUI
void draw_video(unsigned char *src, 
    int dst_x,
    int dst_y,
    int dst_w,
    int dst_h,
    int src_w,
    int src_h,
    int src_rowspan)
{
    unsigned char **dst_rows = gui_bitmap->get_row_pointers();
    int nearest_x[dst_w];
    int nearest_y[dst_h];


// draw the video
    if(landscape || 1)
    {
//printf("draw_video %d dst_w=%d dst_h=%d src_w=%d src_h=%d\n",
//__LINE__, dst_w, dst_h, src_w, src_h);
        for(int i = 0; i < dst_w; i++)
        {
            nearest_x[i] = i * src_w / dst_w;
            CLAMP(nearest_x[i], 0, src_w - 1);
        }
        
        for(int i = 0; i < dst_h; i++)
        {
            nearest_y[i] = i * src_h / dst_h;
            CLAMP(nearest_y[i], 0, src_h - 1);
        }
        
        for(int i = 0; i < dst_h; i++)
        {
            unsigned char *src_row = src + nearest_y[i] * src_rowspan;
            unsigned char *dst_row = dst_rows[i];
            for(int j = 0; j < dst_w; j++)
            {
                int src_x = nearest_x[j];
                *dst_row++ = src_row[src_x * 3 + 0];
                *dst_row++ = src_row[src_x * 3 + 1];
                *dst_row++ = src_row[src_x * 3 + 2];
                dst_row++;
            }
        }
    }
    else
    {
        for(int i = 0; i < dst_w; i++)
        {
            nearest_x[i] = i * src_h / dst_w;
        }
        for(int i = 0; i < dst_h; i++)
        {
            nearest_y[i] = i * src_w / dst_h;
        }
        for(int i = 0; i < dst_h; i++)
        {
            unsigned char *src_col = src + nearest_y[dst_h - i - 1] * 3;
            unsigned char *dst_row = dst_rows[i];
            for(int j = 0; j < dst_w; j++)
            {
                int src_y = nearest_y[j];
                *dst_row++ = src_col[src_y * src_rowspan + 0];
                *dst_row++ = src_col[src_y * src_rowspan + 1];
                *dst_row++ = src_col[src_y * src_rowspan + 2];
                dst_row++;
            }
        }
    }

    gui->lock_window();
    if(gui->need_clear_video)
    {
// printf("draw_video %d x=%d y=%d w=%d h=%d\n",
// __LINE__,
// WINDOW_W - gui->reserved_w,
// 0,
// gui->reserved_w,
// WINDOW_H);
        gui->clear_box(WINDOW_W - gui->reserved_w,
            0, 
            gui->reserved_w,
            WINDOW_H);
        gui->flash(0);
        gui->need_clear_video = 0;
    }

    gui->draw_bitmap(gui_bitmap, 
	    1,
	    dst_x, // dst coords
	    dst_y,
	    dst_w,
	    dst_h,
	    0, // src coords
	    0,
	    dst_w,
	    dst_h);
    gui->unlock_window();
}






