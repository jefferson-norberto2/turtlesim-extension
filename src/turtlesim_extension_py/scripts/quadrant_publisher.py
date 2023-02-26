#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from turtlesim_extension_msgs.msg import Quadrant

class QuadrantPublisher:
    def __init__(self):
        self.current_x = 3.0
        self.current_y = 6.0
        self.pub_interval = 3

        self.timer_pub = rospy.Timer(rospy.Duration(self.pub_interval), self.timer_callback)
        self.quadrant_publisher = rospy.Publisher("/turtlesim_quadrant", Quadrant, queue_size=10)

        rospy.loginfo("Quadrant initialized")

    def timer_callback(self, event):
        quadrant_msg = Quadrant()
        quadrant_msg.current_x = self.current_x
        quadrant_msg.current_y = self.current_y
        self.quadrant_publisher.publish(quadrant_msg)

if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_quadrant")
        QuadrantPublisher()
        rospy.spin()
    except rospy.ROSInterruptException as interrupt:
        rospy.logerr("Error in node number:" + interrupt)