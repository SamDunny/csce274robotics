#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from time import sleep

def square():
    pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
    rospy.init_node('project_3_square')
    msg1 = Twist2DStamped(header=None, v = 0.3, omega = 0.0)
    msg2 = Twist2DStamped(header=None, v = 0.0, omega = 0.2)

    for i in range(0,4):
        pub.publish(msg1)
        sleep(4)
        pub.publish(msg2)
        sleep(1)


    msg3 = Twist2DStamped(header = None, v = 0, omega = 0) # stop robot
    pub.publish(msg3)
    
    
if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException:
        pass

