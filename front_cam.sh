#!/bin/bash
taskset -c 4-7 ./inline /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_DECXIN_CAMERA_01.00.00-video-index0 192.168.101.1:5602,$1 null,$2 /home/ubuntu/video/cam0.ts /dev/shm/cam0 5000 1280 720 1024 576 512 288 90 1000 500 0
