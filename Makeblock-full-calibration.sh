#!/bin/bash

source ~/.bashrc

gnome-terminal  -x $SHELL -ic "roscore"

sleep 3
gnome-terminal  -x $SHELL -ic  'mon launch robot_launch makeblock.launch fullcalib:=true'

sleep 5
gnome-terminal  -x $SHELL -ic  'mon launch orb_slam2_odom_ros odometry_d455.launch'

sleep 5
gnome-terminal -x $SHELL -ic "mon launch robot_launch laser_scan_matcher_calib.launch"


exit 0
