#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
# what more do I need here?

def pid():
#       need: set point = 0, Kp, Ki, Kd
#       need to get input from ?
#       need to publish output to ?

#       previous_error := 0
#       integral := 0

#       loop:
#           error := setpoint − measured_value
#           integral := integral + error × dt
#           derivative := (error − previous_error) / dt
#           output := Kp × error + Ki × integral + Kd × derivative
#           previous_error := error
#           wait(dt)
#           goto loop

    
if __name__ == '__main__':
    try:
        pid()
    except rospy.ROSInterruptException:
        pass

