#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class hw4_pub:
    def __init__(self):
        # publishes to the "/homework1/delta" topic
        self.pub = rospy.Publisher("/homework1/delta", Float32, queue_size=10)
        self.total = 0

    def callback(self, data):
        # keeps running total of input data
        self.total += data.data
        self.pub.publish(self.total)

if __name__ == '__main__':
    # initializing node as 'hw4_pub'
    rospy.init_node('hw4_pub')
    hw4_pub()

    # prevents python from exiting until node is stopped
    rospy.spin()
