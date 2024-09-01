#!/bin/bash
taskset -c 4-7 ./inline /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_DECXIN_CAMERA_01.00.00-video-index0 192.168.101.1:5602,$1 /home/ubuntu/video/cam0.ts /dev/shm/cam0 1024 768 800 600 60 1000 1
