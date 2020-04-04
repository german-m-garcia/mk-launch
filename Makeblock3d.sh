#!/bin/bash

source ~/.bashrc



gnome-terminal  -x $SHELL -ic "roscore"
sleep 5
gnome-terminal  -x $SHELL -ic  'mon launch robot_launch makeblock3d.launch'

# gnome-terminal  -x $SHELL -ic "rosrun odometry splitOdometry.py"


exit 0
