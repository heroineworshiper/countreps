UNAME := $(shell uname)
ARCH := $(shell arch)

ifeq ($(UNAME), Darwin)
# Mac

CFLAGS := \
	-std=c++11 \
	-Iinclude \
        -Llib \
        -lopenpose \
	-lpthread \
        `pkg-config opencv --libs`

CC := clang++


else # Mac
# Linux



OPENPOSE_DIR := /root/openpose
OPENCV_DIR := /root/opencv-3.4.3/build
GUI_DIR := /root/hvirtual/guicast

CFLAGS := \
	-g \
        -O2 \
        -std=c++11 \
	-I$(OPENPOSE_DIR)/include \
	-I$(OPENCV_DIR)/include \
        -I$(GUI_DIR) \
        -I/usr/include/freetype2

LFLAGS := \
        -L$(OPENCV_DIR)/lib \
        -L`pwd`/lib \
        -L$(OPENPOSE_DIR)/build/src/openpose/ \
        -lopenpose \
        `PKG_CONFIG_PATH=$(OPENCV_DIR)/lib/pkgconfig/ pkg-config opencv --libs` \
	-lpthread \
        $(GUI_DIR)/$(ARCH)/libguicast.a \
        -lX11 \
        -lXext \
        -lXft \
        -lXv \
        -lpng \
        -lfreetype
        
CC := g++



endif  # Linux




#AVR_DIR := /root/arduino-1.6.0/hardware/tools/avr/bin/
AVR_DIR := /amazon2/root/arduino-1.8.5/hardware/tools/avr/bin/
AVR_GCC := $(AVR_DIR)avr-gcc
AVR_OBJCOPY := $(AVR_DIR)avr-objcopy -j .text -j .data -O ihex
AVR_DUDE := avrdude -v -patmega328p -cstk500v1 -P/dev/ttyACM0 -b19200
AVR_CFLAGS := -O2 -mmcu=atmega328p
AVR_LFLAGS := -O2 -mmcu=atmega328p -Wl,--section-start=.text=0x0000 -nostdlib


OBJS := \
	countreps.o \
	gui.o 

TRACKER_OBJS := \
        irlib.o \
	tracker.o \
	trackerx11.o \
	trackerserver.o

countreps: $(OBJS)
	$(CC) -o countreps $(OBJS) $(LFLAGS)

tracker: $(TRACKER_OBJS)
	$(CC) -O2 -o tracker $(TRACKER_OBJS) $(LFLAGS)

test: test.c
	gcc -o test test.c


# compile servos
servos:
	$(AVR_GCC) $(AVR_CFLAGS) -o servos.o servos.c avr_debug.c
	$(AVR_GCC) $(AVR_LFLAGS) -o servos.elf servos.o
	$(AVR_OBJCOPY) servos.elf servos.hex

servos_isp:
	$(AVR_DUDE) -Uflash:w:servos.hex:i -Ulock:w:0x0F:m

servos_fuse:
	$(AVR_DUDE) -Ulock:w:0x3F:m -Ulfuse:w:0xE7:m
	$(AVR_DUDE) -Ulock:w:0x3F:m -Uhfuse:w:0xDA:m
	$(AVR_DUDE) -Ulock:w:0x3F:m -Uefuse:w:0x05:m


#clean:
#	rm -f countreps tracker *.o

clean:
	rm -f tracker *.o

$(OBJS):
	$(CC) $(CFLAGS) -c $< -o $*.o

$(TRACKER_OBJS):
	$(CC) $(CFLAGS) -c $< -o $*.o

countreps.o: countreps.c
gui.o: gui.c
irlib.o: irlib.c
tracker.o: tracker.c
trackerx11.o: trackerx11.c
trackerserver.o: trackerserver.c


