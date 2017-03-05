#!/usr/bin/env python
import rospy

from gamecontrol.msg import Motioncmd





def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.command)
	
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/motioncmd", Motioncmd, callback)

	# spin() simply keeps python from exiting until this node is stopped
	print('spinning')
	rospy.spin()
	print('finished')

		
if __name__ == '__main__': 
		
	listener()
