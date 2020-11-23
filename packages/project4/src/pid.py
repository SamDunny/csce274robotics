#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import Twist2DStamped

class Project4:

    def __init__(self):
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

        # starting off with 0 previous error and 0 for the integral value
        self.prevErr_phi = 0
        self.integral_phi = 0

        # setpoint is ideally 0, to maintain lane
        self.setpoint = 0

        # controls
        self.Kp = 1;
        self.Ki = 1;
        self.Kd = 1;


    # gets called everytime lan_pose publishes new data
    def callback(self, data):
        # calling pid_phi for phi variable
        return_val = self.pid_phi(data.phi)
        rospy.logerr("Dunny Demo Time")

        msg = Twist2DStamped(header = None, v = 0, omega = return_val)
        self.pub.publish(msg)


    # pid controller for phi (variable to remain parallel in-between the lines)
    def pid_phi(self, phi):
        error = self.setpoint - phi
        self.integral_phi = self.integral_phi + error
        derivative = error - self.prevErr_phi
        output = (self.Kp * error) + (self.Ki * self.integral_phi) + (self.Kd * derivative)
        self.prevErr_phi = error
        return output
    
    
if __name__ == '__main__':
    rospy.init_node('pid')
    Project4()

    rospy.spin()

