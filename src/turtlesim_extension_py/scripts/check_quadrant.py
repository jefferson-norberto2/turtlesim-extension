#!/usr/bin/env python3

import rospy
from turtlesim_extension_msgs.msg import Quadrant
from turtlesim.msg import Pose
from turtlesim_extension_msgs.srv import CheckQuadrant, CheckQuadrantResponse

class QuadrantPublisher:
    def __init__(self):
        self.check_less_srv = rospy.Service("check_quadrant", CheckQuadrant, self.check_quadrant_srv_callback)
        rospy.loginfo("Check Quadrant initialized")
    
    def check_quadrant_srv_callback(self, request):
        rospy.loginfo("Verificando se é o quadrante")
        res = CheckQuadrantResponse()
        if(request.number == self.quadrant):
            res.result = True
        else:
            res.result = False
        return res

if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_check_quadrant")
        QuadrantPublisher()
        rospy.spin()
    except rospy.ROSInterruptException as interrupt:
        rospy.logerr("Error in node number:" + interrupt)