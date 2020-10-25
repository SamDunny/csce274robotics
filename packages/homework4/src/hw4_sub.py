#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class hw4_sub:
    def __init__(self):
        # subscribes to the topic "/homework/total"
        rospy.Subscriber("/homework1/total", Float32, self.callback)

    def callback(self, data):
        # log each message using the “INFO” warning level
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    # initializing node as 'hw4_sub'
    rospy.init_node('hw4_sub', anonymous=True)
    hw4_sub()

    # prevents python from exiting until node is stopped
    rospy.spin()
