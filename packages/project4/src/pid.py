#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import Twist2DStamped

class Project4:

    def __init__(self):
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

        # starting off with 0 previous error and 0 for the integral value
        self.prevErr_d = 0
        self.integral_d = 0
        self.prevErr_phi = 0
        self.integral_phi = 0

        # setpoint is ideally 0, to maintain lane
        self.setpoint = 0

        # controls
        self.Kp = 1;
        self.Ki = 1;
        self.Kd = 1;


    # gets called everytime lan_pose publishes new data
    def callback(data):
        # calling pid_d for d variable
        pid_d(data.d)
        # calling pid_phi for phi variable
        pid_phi(data.phi)


    # pid controller for d (variable to stay in center of the lane)
    def pid_d(self, d):
        error = self.setpoint - d
        self.integral_d = self.integral_d + error
        derivative = error - self.prevErr_d
        output = (self.Kp * error) + (self.Ki * self.integral_d) + (Kd * derivative_d)
        self.prevErr_d = error
        msg = Twist2DStamped(header = None, v = output, omega = 0)
        self.pub.publish(msg)


    # pid controller for phi (variable to remain parallel in-between the lines)
    def pid_phi(self, phi):
        error = self.setpoint - phi
        self.integral_phi = self.integral_phi + error
        derivative = error - self.prevErr_phi
        output = (self.Kp * error) + (self.Ki * self.integral_phi) + (Kd * derivative_phi)
        self.prevErr_phi = error
        msg = Twist2DStamped(header = None, v = 0, omega = output)
        self.pub.publish(msg)
    
    
if __name__ == '__main__':
    rospy.init_node('pid')
    Project4()

    rospy.spin()

