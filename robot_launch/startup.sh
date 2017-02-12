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
cmds[1]="cd ../launch; roslaunch cameras.launch"

#start the slam node
cmds[2]="roslaunch slam.launch"

#start up the odometry
cmds[3]="cd ~/makeblock/PythonForMegaPi/examples; python encoderOdometry.py "

#start up RVIZ
cmds[4]="rviz -d ~/.rviz/slam.rviz"


for i in 1 2 3; do
  options+=($tab  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"


exit 0
