#!/usr/bin/env python
from megapi import *
import sys
import select
sys.path.insert(1,'/home/tomatito/makeblock/PythonForMegaPi/src')
import threading


import threading
import time
import math
from time import sleep
from apscheduler.scheduler import Scheduler
import logging

#imports for ROS odometry
import numpy as np
import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from gamecontrol.msg import Motioncmd


import atexit


#roslib.load_manifest('odom_publisher')


def exitFunct():
	print('exitFunct()')



class Node:
	def __init__(self):
		rospy.init_node('odometryPublisher', anonymous=True)
		
	
	def __del__(self):
		print('Node destructor')
	
	
	def execute(self):
		rate = rospy.Rate(2) # 2hz
	
		#while 1:
		while not rospy.is_shutdown():
			print('haaallo')
			rate.sleep()
		
	
		
		
if __name__ == '__main__': 
	
	
	
	#create the ROS odometry publisher
	odometryPublisher = Node()
	atexit.register(exitFunct)
	
	try:
		odometryPublisher.execute()
	except rospy.ROSInterruptException:		
		print('exiting')
		pass
	

