#!/usr/bin/env python3

import rospy
from turtlesim_extension_msgs.msg import Quadrant
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, SetPenResponse
from turtlesim_extension_msgs.srv import CheckQuadrant, CheckQuadrantResponse

class QuadrantPublisher:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.t = 0.0
        self.pub_interval = 0.1
        self.quadrant = 0

        self.timer_pub = rospy.Timer(rospy.Duration(self.pub_interval), self.timer_callback)
        self.quadrant_publisher = rospy.Publisher("/turtlesim_quadrant", Quadrant, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback, queue_size=10)
        self.check_less_srv = rospy.Service("check_quadrant", CheckQuadrant, self.check_quadrant_srv_callback)
        self.pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)

        rospy.loginfo("Quadrant initialized")

    def timer_callback(self, event):
        quadrant_msg = Quadrant()
        quadrant_msg.quadrant = self.verify_quadrant()
        self.quadrant = quadrant_msg.quadrant
        self.quadrant_publisher.publish(quadrant_msg)
    
    def pose_callback(self, pose: Pose):
        self.x = pose.x
        self.y = pose.y
        self.t = pose.theta
    
    def check_quadrant_srv_callback(self, request):
        rospy.loginfo("Verificando se é o quadrante")
        res = CheckQuadrantResponse()
        if(request.number == self.quadrant):
            res.result = True
        else:
            res.result = False
        return res
    
    def verify_quadrant(self) -> int:
        if self.x <= 6 and self.y > 6:
            self.pen.call(r=255, g=0, b=0, width=2, off=0)
            return 1
        elif self.x > 6 and self.y > 6:
            self.pen.call(r=0, g=255, b=0, width=2, off=0)
            return 2
        elif self.x <= 6 and self.y <= 6:
            self.pen.call(r=0, g=0, b=255, width=2, off=0)
            return 3
        else:
            self.pen.call(r=255, g=255, b=255, width=2, off=0)
            return 4


if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_quadrant")
        QuadrantPublisher()
        rospy.spin()
    except rospy.ROSInterruptException as interrupt:
        rospy.logerr("Error in node number:" + interrupt)