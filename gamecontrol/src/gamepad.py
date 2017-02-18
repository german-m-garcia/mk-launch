from inputs import get_gamepad
from inputs import get_key
from inputs import get_mouse
import rospy
from gamecontrol.msg import Motioncmd

class GamepadReader():
  
  def __init__(self):
    self.forward = False
    self.backward = False
    self.right = False
    self.left = False
    self.stop = True
    rospy.init_node('gamepad', anonymous=True)
    self.pub = rospy.Publisher('/motioncmd', Motioncmd)
  
  def loop(self):  

	msg = Motioncmd()
	while 1:
	  events = get_gamepad()
	  
	  for event in events:
		if event.ev_type == 'Absolute':
			#movement left-right
			if event.code == 'ABS_X':
			  if event.state == 0:
			    msg.command = 'L'
			    print('move left')
			  if event.state == 128:
			    msg.command = ' '
			    print('stop')
			  if event.state == 255:
			    msg.command = 'R'
			    print('move right')
			#movement forward-backward    
			if event.code == 'ABS_Y':
			  if event.state == 0:
			    msg.command = 'F'
			    print('move forward')
			  if event.state == 128:
			    msg.command = ' '
			    print('stop')
			  if event.state == 255:
			    msg.command = 'B'
			    print('move backward')
			self.pub.publish(msg)   
    
    
	  #print(event.ev_type, event.code, event.state)




if __name__ == '__main__': 

  gamepad = GamepadReader()
  gamepad.loop()
