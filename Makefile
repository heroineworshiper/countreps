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

all: countreps.c
	$(CC) -O2 -o countreps countreps.c $(CFLAGS)








