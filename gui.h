#ifndef GUI_H
#define GUI_H


void init_gui();
void update_gui(unsigned char *src, 
    int w, 
    int h, 
    int reps, 
    int exercise,
    float fps,
    int bgr);
void draw_body(int *xy_array, int decoded_w, int decoded_h);

void finish_gui();
void flash_gui();

#endif

