#!/bin/bash

source ~/.bashrc


#gnome-terminal  -x $SHELL -ic  'mon launch robot_launch remotecontrol.launch'
gnome-terminal  -x $SHELL -ic  'roslaunch robot_launch remotecontrol.launch'


exit 0
