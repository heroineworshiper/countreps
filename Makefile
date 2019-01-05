MAC_CFLAGS := \
	-std=c++11 \
	-Iinclude \
        -Llib \
        -lopenpose \
	-lpthread

MAC_CC := clang++


OBJ_DIR := $(shell uname --machine)
OPENPOSE_DIR := /root/openpose
OPENCV_DIR := /root/opencv-3.4.3/build
OTHER_DIR := /root/colorize/include
LINUX_CFLAGS := \
	-std=c++11 \
	-I$(OPENPOSE_DIR)/include \
	-I$(OPENCV_DIR)/include \
        -I$(OTHER_DIR) \
        -L$(OPENCV_DIR)/lib \
        -L`pwd`/lib \
        -lopenpose \
        -lopencv_core \
	-lpthread


all: countreps_mac.c
	$(MAC_CC) -O2 -o countreps_mac countreps_mac.c $(MAC_CFLAGS)


countreps: countreps.c
	g++ -O2 -o countreps countreps.c $(LINUX_CFLAGS)







