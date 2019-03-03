#!/usr/bin/env python
from inputs import get_gamepad
from inputs import get_key
from inputs import get_mouse
import rospy

from sensor_msgs.msg import Joy
from apscheduler.scheduler import Scheduler

class GamepadReader():
  
  def __init__(self):

    self.y = 0
    self.x = 0
    
    rospy.init_node('gamepad', anonymous=True)
    self.pub = rospy.Publisher('/joystick', Joy)
    self.sched = Scheduler()
    self.sched.start()
    job = self.sched.add_interval_job(self.publish_commands, seconds=0.03, args=[])

  def publish_commands(self):
    joy = Joy()
    joy.axes.append(self.y)
    joy.axes.append(self.x)    
    joy.header.stamp = rospy.Time.now()
    joy.header.frame_id = 'base_link'
    self.pub.publish(joy)  
  
  def loop(self):
		rate = rospy.Rate(30) # 100hz
		
		while not rospy.is_shutdown():
			events = get_gamepad()

			for event in events:
				if event.ev_type == 'Absolute':
					#movement left-right
					if event.code == 'ABS_X':
						if event.state == 0:
							self.y = 0
							self.x = 1
							print('move left')
						if event.state == 128:
							#self.y = 0
							self.x = 0
							#print(' stop ')
						if event.state == 255:
							self.y = 0
							self.x = -1
							print('move right')
					#movement forward-backward    
					if event.code == 'ABS_Y':
						if event.state == 0:
							self.y = 1
							self.x = 0
							print('move forward')
						if event.state == 128:
							self.y = 0
							#self.x = 0
							#print('stop')
						if event.state == 255:
							self.y = -1
							self.x = 0
							print('move backward')
			#publisher.publish(joy)
			#self.publish_commands()
			#rate.sleep()    
			#print(event.ev_type, event.code, event.state)




if __name__ == '__main__': 

  gamepad = GamepadReader()
  try:
  	gamepad.loop()
  except rospy.ROSInterruptException:		
		print('exiting gamepad node')
		pass	
