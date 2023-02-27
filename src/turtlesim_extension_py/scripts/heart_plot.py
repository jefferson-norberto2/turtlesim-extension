#!/usr/bin/env python3

import rospy
from turtlesim_extension_msgs.msg import Quadrant
from turtlesim.msg import Pose

class HeartPlot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.t = 0.0
        self.pub_interval = 1

        self.timer_pub = rospy.Timer(rospy.Duration(self.pub_interval), self.timer_callback)
        self.quadrant_publisher = rospy.Publisher("/turtlesim_quadrant", Quadrant, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback, queue_size=10)

        rospy.loginfo("Quadrant initialized")

    def timer_callback(self, event):
        quadrant_msg = Quadrant()
        quadrant_msg.quadrant = self.verify_quadrant()
        self.quadrant_publisher.publish(quadrant_msg)
    
    def pose_callback(self, pose: Pose):
        self.x = pose.x
        self.y = pose.y
        self.t = pose.theta
    
    def verify_quadrant(self) -> int:
        if self.x <= 6 and self.y > 6:
            return 1
        elif self.x > 6 and self.y > 6:
            return 2
        elif self.x <= 6 and self.y <= 6:
            return 3
        else:
            return 4
    
    def heart_function(self):
        self.x**2+(self.y-(self.x**(2/3)))**2

if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_quadrant")
        HeartPlot()
        rospy.spin()
    except rospy.ROSInterruptException as interrupt:
        rospy.logerr("Error in node number:" + interrupt)