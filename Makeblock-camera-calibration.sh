#!/bin/bash

source ~/.bashrc

gnome-terminal  -x $SHELL -ic "roscore"
sleep 3
gnome-terminal  -x $SHELL -ic  'roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30  align_depth:=true publish_odom_tf:=false'

exit 0
