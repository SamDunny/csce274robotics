#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from time import sleep

def circle():
    pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
    rospy.init_node('project_3_circle')
    msg = Twist2DStamped(header=None, v = 0.3, omega = 0.3)
    pub.publish(msg)

    # robot runs for 10 seconds
    sleep(30)

    # stop robot
    msg = Twist2DStamped(header = None, v = 0, omega = 0)
    pub.publish(msg)
    
    
if __name__ == '__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass

