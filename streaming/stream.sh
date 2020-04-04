#!/bin/bash


# set resolution to 1280x720
# v4l2-ctl --device /dev/video2 --set-fmt-video=width=1280,height=720,pixelformat=UYVY

ffmpeg -y -i /dev/video2 -r 30  -c:v libx264  -preset superfast -tune zerolatency     -x264opts crf=20:vbv-maxrate=3000:vbv-bufsize=100:intra-refresh=1:slice-max-size=1500:keyint=30:ref=1  -pix_fmt yuv420p  -f rtp rtp://192.168.178.84:8080
