#!/usr/bin/env python
'''
Gazebo Robot Controller designed by Team SB_0871
'''

import time
from math import sin as sin
from math import atan2 as atan2
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class PIDController:
    '''
    The Discrete PID controller class. It takes sampling time as input.
    The values of Kp, Ki, Kd are found out using continuous experimentaions.
    limMin and limMax prevent are minimum and maximum possible outputs.
    limMinInt and limMaxInt are saturation limits for integral error.
    tau is a parameter in LOW PASS FILTER cascaded with differentiator.
    Output of PID_update function is the rotational 'z' value of the robot.

    Reference: https://www.youtube.com/watch?v=zOByx3Izf5U&feature=youtu.be
    '''

    integrator = 0
    prev_error = 0
    differentiator = 0
    prev_measurement = 0

    out = 0

    Kp = 10.0
    Ki = 5.0
    Kd = 0.025
    tau = 0.02
    limMin = -10
    limMax = 10
    limMinInt = -4
    limMaxInt = 4

    sampling_time = 0

    def __init__(self, T):
        self.sampling_time = T

    def PID_update(self, setpoint, measurement):

        error = float(setpoint - measurement)

        proportional = self.Kp * error

        self.integrator = self.integrator + \
                          0.5 * self.Ki * self.sampling_time * (error + self.prev_error)

        if self.integrator > self.limMaxInt:

            self.integrator = self.limMaxInt

        elif self.integrator < self.limMinInt:

            self.integrator = self.limMinInt

        self.differentiator = ((-(2.0 * self.Kd * (measurement - self.prev_measurement)
                                  + (2.0 * self.tau - self.sampling_time)*self.differentiator))
                               / (2.0 * self.tau + self.sampling_time))

        self.out = proportional + self.integrator + self.differentiator

        if self.out > self.limMax:

            self.out = self.limMax

        elif self.out < self.limMin:

            self.out = self.limMin

        self.prev_error = error
        self.prev_measurement = measurement

        return self.out



def waypoints(pos):
    '''
    Gives the next point on the wave functioned path.
    A factor 0.65 is added in the current x position to get
    the x and y co-ordinates of the next point in the path.
    '''
    x = pos[0]+0.65
    y = 2*sin(x)*sin(x/2)
    return [x, y]



def odom_callback(data):
    '''
    Odometry callback function.
    Used to process and store output data from /odom topic.
    The x and y coordinates and current heading (angle)
    are stored in global variable POSE
    '''
    global POSE
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    POSE = [data.pose.pose.position.x, data.pose.pose.position.y, \
            euler_from_quaternion([x, y, z, w])[2]]



def laser_callback(msg):
    '''
    Laserscan callback function.
    Used to process and store output data from ebot/laser/scan topic.
    The region of laser is divided into 5 parts
    bleft fleft front fright bright
    The minimum distanced point from each individual area
    is stored in a global variable REGIONS.
    '''
    global REGIONS
    REGIONS = {
        'bright': min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front': min(min(msg.ranges[288:431]), 10),
        'fleft': min(min(msg.ranges[432:575]), 10),
        'bleft': min(min(msg.ranges[576:719]), 10),
    }



def avoid_obstacle(velocity_msg):
    '''
    The Avoid Ostacle function takes velocity_msg (an object of Twist class)
    as input. This algorithm determines the linear x and angular z velociy
    of the robot by comparing its distance from the obstacle.
    RRGIONS variable contains the data from laser sensor.
    '''
    d = 1.7
    if REGIONS['front'] >= 9.5 and REGIONS['fleft'] > d and REGIONS['fright'] > 3:
        velocity_msg.linear.x = 0.4
        velocity_msg.angular.z = - 4
    elif REGIONS['front'] > d and REGIONS['fleft'] > d and REGIONS['fright'] > 3:
        velocity_msg.linear.x = 0.7
        velocity_msg.angular.z = 0
    elif REGIONS['front'] > d and REGIONS['fleft'] > d and REGIONS['fright'] < d:
        velocity_msg.linear.x = 0.7
        velocity_msg.angular.z = 0
    elif REGIONS['front'] < d and REGIONS['fleft'] > d and REGIONS['fright'] < d:
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    elif REGIONS['front'] < d and REGIONS['fleft'] < d and REGIONS['fright'] < d:
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 1
    else:
        pass


def control_loop():
    '''
    The main loop responsible for movement of the robot.
    It contains 2 main algorithms, one for the sine wave path
    and another for obstacle avoidance and path finding.
    '''
    global POSE
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    F = 50.0 # frequency / sampling rate
    rate = rospy.Rate(F)
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    Controller = PIDController(float(1/F)) # PID Controller object

    #######################################################
    ########## Algorithm for sine wave path ###############
    #######################################################
    while not rospy.is_shutdown():
        [x_goal, y_goal] = waypoints(POSE)  # gives next point in the path
        x_cur = POSE[0]                     # current x co-ordinate
        y_cur = POSE[1]                     # current y co-ordinate

        if x_cur >= 6: # approximate final point of sine wave path
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
            break

        head = POSE[2]                      # current heading angle
        wave_goal = atan2((y_goal - y_cur), (x_goal - x_cur))   # angle of next point in the path

        velocity_msg.linear.x = 0.6
        velocity_msg.angular.z = Controller.PID_update(wave_goal, head)

        pub.publish(velocity_msg)
        # print velocity_msg

        rate.sleep()

    time.sleep(0.1)
    print "Controller message pushed at {}".format(rospy.get_time())
    print "Job 1 Done !!"
    print "x =" + str(POSE[0]) + " , y = " + str(POSE[1]) + " , z = " + str(POSE[2])
    time.sleep(0.1)


    #######################################################
    ############# Algorithm for obstacle path #############
    #######################################################

    while not rospy.is_shutdown():

        inc_x = GOAL_X - POSE[0]
        inc_y = GOAL_Y - POSE[1]
        head = POSE[2]
        angle_to_goal = atan2(inc_y, inc_x)

        if POSE[0] < 10.0:
            angle_to_goal = 0.0

        if abs(inc_x - inc_y) < 0.05:
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
            break
        elif (REGIONS['front'] < 2.5 or REGIONS['fright'] < 1.7) and POSE[0] <= 9.5:
            avoid_obstacle(velocity_msg)
        else:
            velocity_msg.linear.x = 0.7
            velocity_msg.angular.z = Controller.PID_update(angle_to_goal, head)

        pub.publish(velocity_msg)
        rate.sleep()

    time.sleep(0.1)
    print "Controller message pushed at {}".format(rospy.get_time())
    print "x =" + str(POSE[0]) + " , y = " + str(POSE[1])
    print "Job 2 Done !!"
    time.sleep(0.1)


'''
All global variables are predfined below.
You can change the final goal point's X and Y co-ordinates below 
and robot will stop at that point.
We have tried to position the robot at different goal points 
after the robot had crossed the concave obstacle.
'''
POSE = [0, 0, 0]
GOAL_X = 12.5   # the X co-ordinate of final goal point
GOAL_Y = 0.0   # the Y co-ordinate of final goal point


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
