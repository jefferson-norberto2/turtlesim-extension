#!/usr/bin/env python3

import rospy
from turtlesim_extension_msgs.msg import Quadrant
from turtlesim.srv import SetPen

class ChangeColorPen:
    def __init__(self):
        self.quadrant_publisher = rospy.Subscriber("/turtlesim_quadrant", Quadrant, self.change_color_pen, queue_size=10)
        self.pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        rospy.loginfo("Change Color Initialized")
    
    def change_color_pen(self, quadrant: Quadrant):
        if quadrant.quadrant == 1:
            self.pen.call(r=255, g=0, b=0, width=2, off=0)
        elif quadrant.quadrant == 2:
            self.pen.call(r=0, g=255, b=0, width=2, off=0)
        elif quadrant.quadrant == 3:
            self.pen.call(r=0, g=0, b=255, width=2, off=0)
        else:
            self.pen.call(r=255, g=255, b=255, width=2, off=0)


if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_change_pen")
        ChangeColorPen()
        rospy.spin()
    except rospy.ROSInterruptException as interrupt:
        rospy.logerr("Error in node number:" + interrupt)