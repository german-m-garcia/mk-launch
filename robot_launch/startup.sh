#!/bin/bash

source ~/.bashrc
roscd robot_launch

tab=" --tab"
options=(--tab)
optionsCore=(--tab)
cmdsCore="roscore"
optionsCore+=($tab  -e "bash -c \"$cmdsCore ; bash\"" ) 
gnome-terminal "${optionsCore[@]}"

sleep 2


#start up the camera nodes
cmds[1]="roslaunch robot_launch cameras.launch"

#start the slam node
cmds[2]="roslaunch robot_launch rtabmap.launch"

#start up the odometry
cmds[3]="roscd odometry; python encoderOdometry.py "

#start up RVIZ
#cmds[4]="rviz -d ~/.rviz/slam.rviz"


for i in 1 2 3; do
  options+=($tab  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"


exit 0
