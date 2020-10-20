#!/usr/bin/env python
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import *


def Waypoints(t):
    x  = t[0] + 0.01 
    y  = 2*sin(x)*sin(x/2)
    # derivative : (cos(x/2)*sin(x) + 2*sin(x/2)*cos(x))
    return [x,y]
    

def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x;
    y = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    #print("x =" + str(pose[0]) + " , y = " + str(pose[1]))
    #print("fnction called")
    pass


def laser_callback(msg):
    # global regions
    # regions = {
    #     'bright':  	,
    #     'fright': 	,
    #     'front':  	,
    #     'fleft':  	,
    #     'bleft':   	,
    # }
    pass

def control_loop():
    global pose

    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():
    	
    	## algorithm
    	[x_goal,y_goal] = Waypoints(pose)
    	x_cur = pose[0]
    	y_cur = pose[1]
    	head = pose[2]
    	goal = atan2((y_goal - y_cur),(x_goal - x_cur))
    	err = goal - head


    	velocity_msg.linear.x = 0.5
        velocity_msg.angular.z = 2*err
    	pub.publish(velocity_msg)
    	#print("Controller message pushed at {}".format(rospy.get_time()))
    	print("x =" + str(pose[0]) + " , y = " + str(pose[1]))
    	rate.sleep()


pose = [0,0,0]

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass


