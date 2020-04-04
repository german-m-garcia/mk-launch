#!/bin/bash

source ~/.bashrc
roscd robot_launch

tab=" --tab"
options=(--tab)
optionsCore=(--tab)
cmdsCore="roscore"
optionsCore+=($tab  -e "bash -c \"$cmdsCore ; bash\"" ) 
gnome-terminal "${optionsCore[@]}"

sleep 1


#start up the camera nodes
cmds[1]="cd ../launch; roslaunch r200_no_ir.launch"


#start the slam node
cmds[2]="sleep 3; rosparam set /camera/driver/r200_emitter_enabled 0"



for i in 1 2; do
  options+=($tab  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"


exit 0

