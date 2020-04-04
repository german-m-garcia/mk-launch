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

#to register  an exit function
import atexit

# whether debug information should be printed
debug = False

class RosOdomPublisher:
	def __init__(self):
		rospy.init_node('odometryPublisher', anonymous=True)
		
		self.gps_ekf_odom_pub = rospy.Publisher('/makeblock/odom', Odometry)
		self.tf_br = tf.TransformBroadcaster()

		self.publish_odom_tf = True

		self.frame_id = 'odom'
		self.child_frame_id = 'base_link'

		# Covariance
		self.P = np.mat(np.diag([0.1]*3))		
		self.P[2,2] = 0.2
		self.vx = self.vy = self.x = self.y = self.z = 0
		self.w = self.theta = 0
		

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

		p_cov = np.array([0.0]*36).reshape(6,6)

		# position covariance
		p_cov[0:2,0:2] = self.P[0:2,0:2]		
		# orientation covariance for Yaw
		# x and Yaw
		p_cov[5,0] = p_cov[0,5] = self.P[2,0]
		# y and Yaw
		p_cov[5,1] = p_cov[1,5] = self.P[2,1]
		# Yaw and Yaw
		p_cov[5,5] = self.P[2,2]
		p_cov[4,4] = self.P[2,2]
		p_cov[3,3] = self.P[2,2]
		p_cov[2,2] = self.P[0,0]
		p_cov[1,1] = self.P[0,0]
		p_cov[0,0] = self.P[0,0]		

		msg.pose.covariance = tuple(p_cov.ravel().tolist())

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
		#print('publishing odometry')		 	

		# Publish odometry message
		self.gps_ekf_odom_pub.publish(msg)

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
		self.x = 0.
		self.y = 0.
		self.theta = 0.
		self.last_theta = 0.
		
		self.vx = 0.
		self.vy = 0.
		self.w = 0.
		self.rosOdometry = rosOdomPublisher
	
	"""
	when moving forward/backward: 0.18 m / 414 tics = 0,00057 m/ tic
	encoder2 increases
	encoder1 decreases
	
	when rotating left: pi/2 in 500 tics => 0.003 rad / tic
	
	
	"""
	def updateTicks(self):
		#update the yaw directly from the encoder
		self.yaw = -self.encoder.getYaw()*2.*3.14159265359/360.
		#print 'arduino yaw=', self.yaw
		curEnc1 = self.encoder.getEnc1()
		if abs(curEnc1 - self.lastEncoder1) > 122:
			if curEnc1 > 122:
				tmp = curEnc1 - 256
			else:
				tmp = 256 -curEnc1
		else:
			tmp = curEnc1
		self.tics1 = (tmp - self.lastEncoder1) 
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
		
		#print 'acc tics1=', self.accTics1
		#print 'acc tics2=', self.accTics2
		
		self.updateOdometry()
		self.rosOdometry.publish_odom(self.x,self.y,self.theta, self.vx, self.vy, self.w)
		
	def rotating(self):
		if self.tics1 > 0 and self.tics2> 0:
			return True
		if self.tics1 < 0 and self.tics2 < 0:
			return True
		return False
		
	
	def updateOdometry(self):
		#we are moving forward or backwards		
		if  sign(self.tics1) != sign(self.tics2):
			self.x = self.x + self.m_tic * (self.tics2) * math.cos(self.theta)
			self.y = self.y + self.m_tic * (self.tics2) * math.sin(self.theta)
			
			#the angle can also change here
			self.theta = self.yaw
		
		else:
			self.theta = self.yaw
			
		#update the velocities
		self.vx = self.m_tic * (self.tics2) * math.cos(self.theta) / self.seconds
		self.vy = self.m_tic * (self.tics2) * math.sin(self.theta) / self.seconds
		self.w = (self.theta - self.last_theta) / self.seconds
		self.last_theta = self.theta
		
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
		self.forward = 'F'
		self.left = 'L'
		self.right = 'R'
		self.stop = ' '
		self.backward = 'B'
		self.command = ' '
		self.subscriber = rospy.Subscriber("/motioncmd", Motioncmd, self.callback)
		self.last_received = rospy.Time.now()
		#check whether no command has been received
		job = self.sched.add_interval_job(self.check_commands, seconds=0.05, args=[])

	
	def check_commands(self):
		now = rospy.Time.now()
		if now - self.last_received > 1000:
			self.command = ' '
		
	
	def callback(self,msg):
		#rospy.loginfo("motion command= %s", msg.command)
		self.last_received = rospy.Time.now()
		self.command = msg.command

	def getCommand(self):
		return self.command
		
	

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


	#if True:
	#	while elapsed_time < 0.5:
	#		sleep(0.02)
	#		elapsed_time = time.time() - start_time
	#		#print elapsed_time	
	#		ser.write(b'F')
		
		#start_time = time.time()
		#elapsed_time = time.time() - start_time
		#while elapsed_time < 0.5:
		#	sleep(0.02)
		#	elapsed_time = time.time() - start_time
		#	#print elapsed_time	
		#	ser.write(b'R')

	
	#	quit()

	
	try:
		while not rospy.is_shutdown():
	
			cmd = readControls.getCommand()		
		
			if cmd == 'F':
				ser.write(b'F')
			elif cmd == 'B':
				ser.write(b'B')
			elif cmd == 'L':
				ser.write(b'L')
			elif cmd == 'R':
				ser.write(b'R')
			else:
				ser.write(b' ')
			if debug:
				print('command read in main loop=', cmd)
				
		
			#if threadKeyboard.getCommand() == 'F':
			#	ser.write(b'F')
			#elif threadKeyboard.getCommand() == 'B':
			#	ser.write(b'B')
			#elif threadKeyboard.getCommand() == 'L':
			#	ser.write(b'L')
			#elif threadKeyboard.getCommand() == 'R':
			#	ser.write(b'R')
			#else:
			#	ser.write(b' ')
			
		
			sleep(0.05)
	except rospy.ROSInterruptException:		
		print('exiting')
		pass	
		
		
		
		#sleep(1)
	
	
	
	#bot = MegaPi()
	#bot.start()
	#while 1:
	#	continue;
