#!/usr/bin/env python
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point , Twist
from tf.transformations import euler_from_quaternion
from math import *


class PIDController:
	integrator=0
	prevError=0
	differentiator=0
	prevMeasurement=0

	out=0

	Kp = 12.0
	Ki = 5.0
	Kd = 0.025
	tau = 0.02
	limMin = -100
	limMax = 100
	limMinInt = -100
	limMaxInt = 100

	T=0

	def __init__(self,T):
		self.T = T

	def PIDupdate(self,setpoint,measurement):

		error = float(setpoint - measurement)

		proportional = self.Kp * error

		self.integrator = self.integrator + 0.5 * self.Ki * self.T * (error + self.prevError)

		if (self.integrator > self.limMaxInt):

			self.integrator = self.limMaxInt

		elif (self.integrator < self.limMinInt):

			self.integrator = self.limMinInt

		self.differentiator =( (-(2.0 * self.Kd * (measurement - self.prevMeasurement) 
							+ (2.0 * self.tau - self.T) * self.differentiator)) / (2.0 * self.tau + self.T) )

		self.out = proportional + self.integrator + self.differentiator

		if (self.out > self.limMax):

			self.out = self.limMax

		elif (self.out < self.limMin):

			self.out = self.limMin

		self.prevError       = error
		self.prevMeasurement = measurement

		return self.out


def Waypoints(t):
	x = t[0]+0.7
	y = 2*sin(x)*sin(x/2)
	# derivative : (cos(x/2)*sin(x) + 2*sin(x/2)*cos(x))
	return [x,y]
    

def odom_callback(data):
	global pose
	x = data.pose.pose.orientation.x;
	y = data.pose.pose.orientation.y;
	z = data.pose.pose.orientation.z;
	w = data.pose.pose.orientation.w;
	pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
	pass


def laser_callback(msg):
	global regions
	# print msg.ranges[360]
	regions = {
		'bright':  min(min(msg.ranges[0:143]), 10)   ,
		'fright':  min(min(msg.ranges[144:287]), 10) ,
		'front':   min(min(msg.ranges[288:431]), 10) ,
		'fleft':   min(min(msg.ranges[432:575]), 10) ,
		'bleft':   min(min(msg.ranges[576:719]), 10) ,
	}
	pass


def avoid_obstacle(velocity_msg):
	d = 1.7
	if regions['front'] >= 9.5 and regions['fleft'] > d and regions['fright'] > 3:
		velocity_msg.linear.x = 0.3
		velocity_msg.angular.z = - 4
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
		velocity_msg.linear.x = 0.7
		velocity_msg.angular.z = 0
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		velocity_msg.linear.x = 0.2
		velocity_msg.angular.z = 0.6
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		velocity_msg.linear.x = 0.5
		velocity_msg.angular.z = 0
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
		velocity_msg.linear.x = 0
		velocity_msg.angular.z = 0.7
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		velocity_msg.linear.x = 0
		velocity_msg.angular.z = 0.7
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		velocity_msg.linear.x = 0
		velocity_msg.angular.z = 0.6
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		velocity_msg.linear.x = 0
		velocity_msg.angular.z = 0.6
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
		velocity_msg.linear.x = 0
		velocity_msg.angular.z = 0.6
	else:
		rospy.loginfo(regions)

def control_loop():
	global pose
	global goal

	rospy.init_node('ebot_controller')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
	rospy.Subscriber('/odom', Odometry, odom_callback)

	F = 50	# Frequency = 10 HZ
	rate = rospy.Rate(F) 
	time.sleep(1)
	msg = LaserScan()
	
	velocity_msg = Twist()
	velocity_msg.linear.x = 0
	velocity_msg.linear.y = 0
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)

	err_prev = 0

	Controller = PIDController(1/F)

	#####################################
	## Algorithm for sine wave path ######
	#####################################
	while not rospy.is_shutdown():
		[x_goal,y_goal] = Waypoints(pose)
		x_cur = pose[0]
		y_cur = pose[1]

		if x_cur >= 6:
			velocity_msg.linear.x = 0
			velocity_msg.linear.y = 0
			velocity_msg.angular.z = 0
			pub.publish(velocity_msg)
			break

		head = pose[2]
		wave_goal = atan2((y_goal - y_cur),(x_goal - x_cur))

		velocity_msg.linear.x = 0.6
		velocity_msg.angular.z = Controller.PIDupdate(wave_goal,head)

		pub.publish(velocity_msg)
		#print("x =" + str(x_cur) + " , y = " + str(y_cur))
		print(velocity_msg)

		rate.sleep()

	time.sleep(0.25)
	print("Job 1 Done !!")
	time.sleep(0.25)

	#####################################
	## Algorithm for obstacle path ######
	#####################################

	while not rospy.is_shutdown():
		inc_x = goal.x - pose[0]
		inc_y = goal.y - pose[1]
		head = pose[2]
		angle_to_goal = atan2(inc_y, inc_x)
		print angle_to_goal - pose[2]

		if inc_x - inc_y < 0.05 and inc_x -inc_y > -0.05:
			velocity_msg.linear.x = 0
			velocity_msg.linear.y = 0
			velocity_msg.angular.z = 0
			pub.publish(velocity_msg)
			break
		elif regions['front'] < 2 or regions['bright'] < 2.5:
			avoid_obstacle(velocity_msg)
		elif abs(angle_to_goal - pose[2]) > 0.1:
			velocity_msg.linear.x = 0.6
			velocity_msg.angular.z = Controller.PIDupdate(angle_to_goal,head)
		else:
			velocity_msg.linear.x = 0.8
			velocity_msg.angular.z = 0.0

		pub.publish(velocity_msg)
		print("Controller message pushed at {}".format(rospy.get_time()))
		print("x =" + str(pose[0]) + " , y = " + str(pose[1]))
		rate.sleep()

	time.sleep(0.25)
	print("Job 2 Done !!")
	time.sleep(0.25)


pose = [0,0,0]
goal = Point()
goal.x = 12.5
goal.y = 0

if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass
