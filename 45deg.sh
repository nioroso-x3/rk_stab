#!/bin/bash
taskset -c 4-7 ./inline /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0 192.168.101.1:5603,$1 null,$2 /home/ubuntu/video/cam1.ts /dev/shm/cam1 5001 1600 1200 1280 960 640 480 30 1000 500 0
