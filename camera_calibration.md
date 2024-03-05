# launch camera
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30  align_depth:=true publish_odom_tf:=false

# launch orb_slam
roslaunch orb_slam2_odom_ros odometry_d455.launch

# launch odometry: camera_link should be detached from the robot