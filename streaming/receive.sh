#!/bin/bash
gst-launch-1.0 udpsrc port=8080 !   "application/x-rtp, payload=127" !    rtph264depay !     avdec_h264 !     videoconvert  ! xvimagesink sync=false
