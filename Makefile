UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)

CFLAGS := \
	-std=c++11 \
	-Iinclude \
        -Llib \
        -lopenpose \
	-lpthread \
        `pkg-config opencv --libs`

CC := clang++


else




OPENPOSE_DIR := /root/openpose
OPENCV_DIR := /root/opencv-3.4.3/build
OTHER_DIR := /root/colorize/include
CFLAGS := \
	-std=c++11 \
	-I$(OPENPOSE_DIR)/include \
	-I$(OPENCV_DIR)/include \
        -I$(OTHER_DIR) \
        -L$(OPENCV_DIR)/lib \
        -L`pwd`/lib \
        -lopenpose \
        -lopencv_core \
	-lpthread
CC := g++
endif

#AVR_DIR := /root/arduino-1.6.0/hardware/tools/avr/bin/
AVR_DIR := /amazon2/root/arduino-1.8.5/hardware/tools/avr/bin/
AVR_GCC := $(AVR_DIR)avr-gcc
AVR_OBJCOPY := $(AVR_DIR)avr-objcopy -j .text -j .data -O ihex
AVR_DUDE := avrdude -v -patmega328p -cstk500v1 -P/dev/ttyACM0 -b19200
AVR_CFLAGS := -O2 -mmcu=atmega328p
AVR_LFLAGS := -O2 -mmcu=atmega328p -Wl,--section-start=.text=0x0000 -nostdlib

all: countreps.c
	$(CC) -O2 -o countreps countreps.c $(CFLAGS)

tracker: tracker.c
	$(CC) -O2 -o tracker tracker.c $(CFLAGS)


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




