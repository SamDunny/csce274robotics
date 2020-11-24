#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import Twist2DStamped

class Project4:

    def __init__(self):
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

        # starting off with 0 previous error
        self.prevErr_phi = 0

        # setpoint is ideally 0, to maintain lane
        self.setpoint = 0

        # controls
        self.Kp = 2
        self.Kd = 3

        # time derivative
        self.dt = 0.1


    # gets called everytime lan_pose publishes new data
    def callback(self, data):
        # calling pid_phi for phi variable
        return_val = self.pid_phi(data.phi)

        # capping return values
        if return_val > 4.0:
            return_val = 4.0
        if return_val < -4.0:
            return_val = -4.0

        rospy.logerr('DUNNY Demo: PID Return Value = {}'.format(return_val) )

        msg = Twist2DStamped(header = None, v = 0.12, omega = return_val)
        self.pub.publish(msg)


    # pid controller for phi (variable to remain parallel in-between the lines)
    def pid_phi(self, phi):
        error = self.setpoint - phi
        derivative = error - self.prevErr_phi
        output = (self.Kp * error) + (self.Kd * (derivative / self.dt))
        self.prevErr_phi = error
        return output
    
    
if __name__ == '__main__':
    rospy.init_node('pid')
    Project4()

    rospy.spin()

