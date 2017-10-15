#!/usr/bin/env python
from inputs import get_gamepad
from inputs import get_key
from inputs import get_mouse
import rospy
from gamecontrol.msg import Motioncmd
from apscheduler.scheduler import Scheduler

class GamepadReader():
  
  def __init__(self):

    self.forward = False
    self.backward = False
    self.right = False
    self.left = False
    self.stop = True
    rospy.init_node('gamepad', anonymous=True)
    self.pub = rospy.Publisher('/motioncmd', Motioncmd)
    self.command = ' '
    self.sched = Scheduler()
    self.sched.start()
    job = self.sched.add_interval_job(self.publish_commands, seconds=0.05, args=[])

  def publish_commands(self):
    msg = Motioncmd()
    msg.command = self.command
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = 'base_link'
    self.pub.publish(msg)  
  
  def loop(self):  

	rate = rospy.Rate(100) # 100hz
	while not rospy.is_shutdown():
	  events = get_gamepad()  
	  
	  for event in events:
		if event.ev_type == 'Absolute':
			#movement left-right
			if event.code == 'ABS_X':
			  if event.state == 0:
			    self.command = 'L'
			    print('move left')
			  if event.state == 128:
			    self.command = ' '
			    print('stop')
			  if event.state == 255:
			    self.command = 'R'
			    print('move right')
			#movement forward-backward    
			if event.code == 'ABS_Y':
			  if event.state == 0:
			    self.command = 'F'
			    print('move forward')
			  if event.state == 128:
			    self.command = ' '
			    print('stop')
			  if event.state == 255:
			    self.command = 'B'
			    print('move backward')
	  rate.sleep()    
	  #print(event.ev_type, event.code, event.state)




if __name__ == '__main__': 

  gamepad = GamepadReader()
  try:
  	gamepad.loop()
  except rospy.ROSInterruptException:		
		print('exiting gamepad node')
		pass	
