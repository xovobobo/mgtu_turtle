#!/usr/bin/env python
import rospy
import json
import time
from mgtu_pack.cfg import my_param_serverConfig
from mgtu_pack.srv import my_srv
from dynamic_reconfigure.server import Server

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class MgtuTurtle:
    def __init__(self) -> None:
        self.pose = Pose()
        Server(my_param_serverConfig,  self.reconfigure_cb)
        self.service = rospy.Service("/my_service", my_srv, self.srv_cb)
        self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    def callback_pose(self, msg: Pose):
        self.pose = msg

    def reconfigure_cb(self, config: my_param_serverConfig, level):
        self.kp = config["kp"]
        self.eps = config["eps"]
        return config

    def srv_cb(self, msg):
        if msg.action == msg.START:
            result = self.turtle_go([msg.x, msg.y])
        return result

    def turtle_go(self, target: list, kp=1, eps=0.1):
        road_done = False
        while not road_done:
            dt_x = target[0] - self.current_pose.x
            dt_y = target[1] - self.current_pose.y
            dt_z = 0 - self.current_pose.theta

            if (abs(dt_x) < eps) and (abs(dt_y) < eps):
                road_done = True

            msg = Twist()
            msg.linear.x = dt_x * self.kp
            msg.linear.y = dt_y * self.kp
            msg.angular.z = dt_z * self.kp
            self.publisher.publish(msg)
            time.sleep(0.1)

        return True

if __name__ == '__main__':
    rospy.init_node('control_node')
    mgtu_turtle = MgtuTurtle()
    rospy.spin()