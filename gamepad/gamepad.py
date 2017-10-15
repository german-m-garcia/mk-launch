from inputs import get_gamepad
from inputs import get_key
from inputs import get_mouse



class GamepadReader():
  
  def __init__(self):
    self.forward = False
    self.backward = False
    self.right = False
    self.left = False
    self.stop = True
  
  def loop(self):  

	while not rospy.is_shutdown():
	  events = get_gamepad()
	  for event in events:
		if event.ev_type == 'Absolute':
			#movement left-right
			if event.code == 'ABS_X':
			  if event.state == 0:
			    print('move left')
			  if event.state == 128:
			    print('stop')
			  if event.state == 255:
			    print('move right')
			#movement forward-backward    
			if event.code == 'ABS_Y':
			  if event.state == 0:
			    print('move forward')
			  if event.state == 128:
			    print('stop')
			  if event.state == 255:
			    print('move backward')   
    
    
	  #print(event.ev_type, event.code, event.state)




if __name__ == '__main__': 

  gamepad = GamepadReader()
  gamepad.loop()
