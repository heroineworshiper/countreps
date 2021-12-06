#insmod /lib/modules/4.9.39/kernel/drivers/video/nvidia-uvm.ko

# redirect stderr to /dev/null
LD_LIBRARY_PATH=/root/openpose/build/src/openpose/ ./tracker 2> /dev/null
#LD_LIBRARY_PATH=/root/openpose/build/src/openpose/ ./tracker 
