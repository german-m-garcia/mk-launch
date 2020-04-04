#!/usr/bin/env python

import sys
import select
sys.path.insert(1,'/home/tomatito/ws/src/makeblock/PythonForMegaPi/src')
from megapi import *
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


from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

#to register  an exit function
import atexit
import covariances

# to pack floats
import struct

# whether debug information should be printed
debug = False

def map(v, in_min, in_max, out_min, out_max):
	# Check that the value is at least in_min
	if v < in_min:
		v = in_min
	# Check that the value is at most in_max
	if v > in_max:
		v = in_max
	return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def joystickToDiff(x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):
	# If x and y are 0, then there is not much to calculate...
	if x == 0 and y == 0:
		return (0, 0)
    

	# First Compute the angle in deg
	# First hypotenuse
	z = math.sqrt(x * x + y * y)

	# angle in radians
	rad = math.acos(math.fabs(x) / z)

	# and in degrees
	angle = rad * 180 / math.pi

	# Now angle indicates the measure of turn
	# Along a straight line, with an angle o, the turn co-efficient is same
	# this applies for angles between 0-90, with angle 0 the coeff is -1
	# with angle 45, the co-efficient is 0 and with angle 90, it is 1

	tcoeff = -1 + (angle / 90) * 2
	turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
	turn = round(turn * 100, 0) / 100

	# And max of y or x is the movement
	mov = max(math.fabs(y), math.fabs(x))

	# First and third quadrant
	if (x >= 0 and y >= 0) or (x < 0 and y < 0):
		rawLeft = mov
		rawRight = turn
	else:
		rawRight = mov
		rawLeft = turn

	# Reverse polarity
	if y < 0:
		rawLeft = 0 - rawLeft
		rawRight = 0 - rawRight

	# minJoystick, maxJoystick, minSpeed, maxSpeed
	# Map the values onto the defined rang
	rightOut = map(-rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
	leftOut = map(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

	return (rightOut, leftOut)


class RosOdomPublisher:
	def __init__(self):
		rospy.init_node('odometryPublisher', anonymous=True)
		
		self.odom_pub = rospy.Publisher('/makeblock/odom', Odometry)
		self.imu_pub = rospy.Publisher('/makeblock/imu', Imu)
		self.tf_br = tf.TransformBroadcaster()

		self.publish_odom_tf = False

		self.frame_id = 'odom'
		self.child_frame_id = 'base_link'
		
		self.vx = self.vy = self.x = self.y = self.z = 0
		self.w = self.theta = 0
		

	def publish_imu(self,imu_theta,imu_w):
		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = self.child_frame_id # i.e. '/base_link'
		
		
		imu_msg.orientation = Quaternion(*(kdl.Rotation.RPY(0, 0, imu_theta).GetQuaternion()))
		imu_msg.orientation_covariance = covariances.ODOM_COV_IMU
		
		imu_msg.angular_velocity.x = 0
		imu_msg.angular_velocity.y = 0
		imu_msg.angular_velocity.z = imu_w
		
		imu_msg.angular_velocity_covariance = covariances.TWIST_COV_IMU		
		self.imu_pub.publish(imu_msg)
		
	
	def publish_odom(self,x,y,theta, vx, vy, w):

		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = self.frame_id # i.e. '/odom'
		msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'
		self.x = x
		self.y = y
		self.theta = theta
		
		self.vx = vx
		self.vy = vy
		self.w = w

		#pose part
		msg.pose.pose.position = Point(self.x, self.y, self.z)
		msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0, 0, self.theta).GetQuaternion()))

		msg.pose.covariance = covariances.ODOM_COV_WHEEL #tuple(p_cov.ravel().tolist())

		pos = (msg.pose.pose.position.x,
			 msg.pose.pose.position.y,
			 msg.pose.pose.position.z)

		ori = (msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.y,
			 msg.pose.pose.orientation.z,
			 msg.pose.pose.orientation.w)
		
			 
		#twist part
		msg.twist.twist.linear.x = self.vx
		msg.twist.twist.linear.y = self.vy
		msg.twist.twist.angular.z  = self.w
		msg.twist.covariance = covariances.TWIST_COV_WHEEL
		
		# Publish odometry message
		self.odom_pub.publish(msg)


		# Also publish tf if necessary
		if self.publish_odom_tf:
		  self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)


"""
encoder and raw odometry classes
"""
exitFlag = 0

sign = lambda a: (a>0) - (a<0)

"""
reads the encoder values periocally and publishes the estimated odometry
"""
class odometryThread(threading.Thread):
	def __init__(self,threadID, name,encoder,rosOdomPublisher):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.sched = Scheduler()
		self.sched.start()        # start the scheduler
		self.encoder = encoder		
		self.lastEncoder1 = 0
		self.lastEncoder2 = 0
		self.tics1 = 0
		self.tics2 = 0
		self.accTics1 = 0
		self.accTics2 = 0
		self.seconds = 0.04 #0.05 ms = time interval in which odometry is checked
		logging.basicConfig()
		
		#odometry: pose=(x,y,theta)
		self.m_tic = 0.00038 #meters per tic
		self.rad_tic = 0.003	#radians per tic
		self.b = 0.174
		self.x = 0.
		self.y = 0.
		self.theta = 0.
		self.last_theta = 0.
			
		
		#velocities
		self.vx = 0.
		self.vy = 0.
		self.w = 0.
		self.rosOdometry = rosOdomPublisher
		
		#imu
		self.imu_theta = 0.
		self.imu_w = 0.
		self.imu_last_theta = 0.
	
	"""
	when moving forward/backward: 0.18 m / 414 tics = 0,00057 m/ tic
	encoder2 increases
	encoder1 decreases
	
	when rotating left: pi/2 in 500 tics => 0.003 rad / tic
	
	
	"""
	def updateTicks(self):
		#update the yaw directly from the encoder
		self.yaw = -self.encoder.getYaw()*2.*3.14159265359/360.
		self.imu_theta = self.yaw

		curEnc1 = self.encoder.getEnc1()
		if abs(curEnc1 - self.lastEncoder1) > 122:
			if curEnc1 > 122:
				tmp = curEnc1 - 256
			else:
				tmp = 256 -curEnc1
		else:
			tmp = curEnc1
		self.tics1 = -(tmp - self.lastEncoder1) 
		self.lastEncoder1 = curEnc1
		
		#print 'odometry tics encoder 1:', self.tics1
		
		curEnc2 = self.encoder.getEnc2()
		if abs(curEnc2 - self.lastEncoder2) > 122:
			if curEnc2 > 122:
				tmp = curEnc2 - 256
			else:
				tmp = 256 - curEnc2
		else:
			tmp = curEnc2
		self.tics2 = (tmp - self.lastEncoder2)
		self.lastEncoder2 = curEnc2
		#print 'odometry tics encoder 2:', self.tics2
		
		#update the accumulated tics
		self.accTics1 = self.accTics1 + self.tics1
		self.accTics2 = self.accTics2 + self.tics2 
		
		#print 'tics1=', self.tics1
		#print 'tics2=', self.tics2
		
		self.updateOdometry()
		self.rosOdometry.publish_odom(self.x,self.y,self.theta, self.vx, self.vy, self.w)
		self.rosOdometry.publish_imu(self.imu_theta, self.imu_w)
		
	def rotating(self):
		if self.tics1 > 0 and self.tics2> 0:
			return True
		if self.tics1 < 0 and self.tics2 < 0:
			return True
		return False
		
	
	def updateOdometry(self):
		
		delta_Ul = self.m_tic * self.tics2
		delta_Ur = self.m_tic * self.tics1
		delta_Ui = (delta_Ul+delta_Ur)/2.
		delta_theta = (delta_Ur-delta_Ul)/self.b
		
		
		#print 'delta_Ul=', delta_Ul
		#print 'delta_Ur=', delta_Ur
		
		
		self.x = self.x + delta_Ui * math.cos(self.theta)
		self.y = self.y + delta_Ui * math.sin(self.theta)
			
		
		self.theta = self.theta + delta_theta
		
			
		#update the velocities
		self.vx = delta_Ui / self.seconds
		self.vy = 0.
		self.w = (self.theta - self.last_theta) / self.seconds
		self.last_theta = self.theta
		
		self.imu_w = (self.imu_theta - self.imu_last_theta) / self.seconds
		self.imu_last_theta = self.imu_theta
		
		#print('updated odometry')
		#we are rotating left
		#elif self.tics1 <=0 and self.tics2 <= 0:
		#	self.theta = self.theta + (-self.tics2) * self.rad_tic
		#we are rotating right
		#elif self.tics1 >= 0 and self.tics2 >= 0:
		#	self.theta = self.theta + (-self.tics2) * self.rad_tic
		
		#print 'x=', self.x
		#print 'y=', self.y
		#print 'theta=', self.theta
	
	def run(self):
		#job = self.sched.add_interval_job(self.updateTicks, seconds=0.04, args=[])
		job = self.sched.add_interval_job(self.updateTicks, seconds = self.seconds, args=[])
 		
		
		
	
"""
Odometry values recorded at pwm speed=80
when moving forward:s 18cm/1s = 0.18 m/s
whem moving bacwards: 21 cm/1s = 0.21 m/s

"""

class encoderThread (threading.Thread):
    def __init__(self, threadID, name, serial):
		threading.Thread.__init__(self)
		self.threadID =	threadID
		self.name = name
		self.serial = serial
		self.encoder1 = 0
		self.encoder2 = 0
		self.yaw=0.
		self.last_yaw = 0.
        
       
    def __del__(self):
    	print('shutting down serial port')
    	self.serial.close()
    
    def getEnc2(self):
    	return self.encoder2
    	
    def getEnc1(self):
    	return self.encoder1

    def getYaw(self):
	return self.yaw
    
    def addTics(self):
    	interval1 = self.encoder1 - self.lastEncoder1
    	self.lastEncoder1 = self.encoder1
    
    def run(self):
        #print "Starting " + self.name
        #while 1:
        while not rospy.is_shutdown():		
		n = ser.inWaiting()
		if n > 0:
			#print  n,' bytes available', '\n'
			typedata = ser.read(1)
			#print 'incoming type=', struct.unpack('B', typedata[0])[0]
			
			n = ser.inWaiting()
			while n < 4:		
				n = ser.inWaiting()	
			data = ser.read(4)
			self.encoder1 = struct.unpack('B', data[0])[0]
			
			#print 'encoder 1=',  self.encoder1, '\n'
			
			
			typedata = ser.read(1)
			#print 'incoming type=', struct.unpack('B', typedata[0])[0]
			n = ser.inWaiting()
			while n < 4:		
				n = ser.inWaiting()	
			data = ser.read(4)
			self.encoder2 = struct.unpack('B', data[0])[0]
			#print 'encoder 2=',  self.encoder2, '\n'

			typedata = ser.read(1)
			#print 'incoming type=', struct.unpack('B', typedata[0])[0]
			n = ser.inWaiting()
			while n < 4:		
				n = ser.inWaiting()	
			datayaw = ser.read(4)
			
			cur_yaw = struct.unpack('f', datayaw)[0]
			update_yaw = (cur_yaw - self.last_yaw)
			#print 'update_yaw0=' ,update_yaw
			if abs(update_yaw) < 0.04:
				update_yaw = 0.0
			#print 'update_yaw1=' ,update_yaw
			#update_yaw = 0.0			
			#print 'update_yaw2=' ,update_yaw
			self.yaw = self.yaw +  update_yaw
			#print 'update_yaw3=' ,update_yaw, ' cur_yaw=', cur_yaw, ' last_yaw=', self.last_yaw , 'yaw=',  self.yaw, '\n'
			self.last_yaw = cur_yaw

			
			
		sleep(0.01)
   
        #print_time(self.name, self.counter, 5)
        #print "Exiting " + self.name

	



class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

	

class ReadControls(threading.Thread):

	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.sched = Scheduler()
		self.sched.start()        # start the scheduler
		self.x = 0
		self.y = 0		
		self.subscriber = rospy.Subscriber("/joystick", Joy, self.callback)
		self.last_received = rospy.Time.now()
		#check whether no command has been received
		job = self.sched.add_interval_job(self.check_commands, seconds=0.05, args=[])

	
	def check_commands(self):
		now = rospy.Time.now()
		if now - self.last_received > 1000:
			self.x = 0
			self.y = 0
		
	
	def callback(self,msg):
		#rospy.loginfo("motion command= %s", msg.command)
		self.last_received = rospy.Time.now()
		
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.axes)
		(rightOut, leftOut) = joystickToDiff(msg.axes[0], msg.axes[1], -1, 1, -125., 125)
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", (rightOut, leftOut))
		self.x = int(leftOut)
		self.y = int(rightOut)

	def getCommand(self):
		return (self.y, self.x)
		
	

class keyboardReadThread(threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.forward = 'F'
		self.left = 'L'
		self.right = 'R'
		self.stop = ' '
		self.backward = 'B'
		self.command = ' '
	def run(self):    
		while 1:
			key = getch()
			charkey = struct.unpack('B', key[0])[0]				
			if key == ' ':
				self.command = self.stop
			if charkey == 119:
				self.command = self.forward				
			elif charkey == 115:
				self.command = self.backward
							
			elif charkey == 97:
				self.command = self.left
				
			elif charkey == 100:
				self.command = self.right
				
			else:			
				self.command = self.stop
				

	def getCommand(self):
		return self.command	


if __name__ == '__main__': 
	
	
	ser = serial.Serial('/dev/ttyUSB0',115200,timeout=10) # open serial port	
	print (ser.name) # check which port was really used
	
	#read the version data
	version = 20
	read = 0
	n = ser.inWaiting()
	while read < version:
		n = ser.inWaiting()
		if n > 0:
			print  n,' bytes available'
			ser.read(n)
			read = read + n
	print "version # read\n"
	
	#create the ROS odometry publisher
	odometryPublisher = RosOdomPublisher()
	
	# Create new threads
	threadEncoder = encoderThread(1, "Thread-encoder", ser)
	threadOdom = odometryThread(2,"Thread-odometry", threadEncoder,odometryPublisher)
	#threadKeyboard = keyboardReadThread(3, "Thread-keyboard")
	
	readControls = ReadControls(3, 'Thread-readcontrols')
		
	
	#start the threads
	threadEncoder.start()	
	threadOdom.start()	
	#threadKeyboard.start()
	
	iterations = 0	
	start_time = time.time()
	elapsed_time = time.time() - start_time

	
	try:
		while not rospy.is_shutdown():
	
			cmd = readControls.getCommand()

			ser.write(struct.pack('b', cmd[0]))
			ser.write(struct.pack('b', cmd[1]))
			
		
			sleep(0.05)
	except rospy.ROSInterruptException:		
		print('exiting')
		pass	
		
		
		
		#sleep(1)
	
	
	
	#bot = MegaPi()
	#bot.start()
	#while 1:
	#	continue;
