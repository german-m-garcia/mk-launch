#!/bin/bash

source ~/.bashrc

gnome-terminal  -x $SHELL -ic "roscore"
sleep 4
gnome-terminal  -x $SHELL -ic  'mon launch robot_launch makeblock.launch'

sleep 1;
#gnome-terminal  -x $SHELL -ic "source /home/tomatito/ws/devel_isolated/setup.bash; cd ~/ws/src/cartographer_ros/cartographer_ros/launch; mon launch makeblock.launch";
gnome-terminal -x $SHELL -ic 'mon launch robot_launch cartographer.launch'


exit 0
