#!/usr/bin/env python
import rospy
import json
import time
from mgtu_pack.cfg import my_param_serverConfig
from mgtu_pack.srv import my_srv
from dynamic_reconfigure.server import Server

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

current_pose = Pose()

def reconfigure_cb(config: my_param_serverConfig, level):
    target = [config["target_x"], config["target_y"]]

    if config["send_cmd"]:
        turtle_go(target,)
        config["send_cmd"] = False

    if config["send_cmd_road"]:
        
        try:
            with open(config["road_file_path"], "r") as file:
                roads = json.load(file)
                for road in roads.values():
                    rospy.loginfo(f"go to {road}")
                    turtle_go(road)
                
        except:
            rospy.logerr("Cant open road file")
        
        config["send_cmd_road"] = False
    
    
    return config

def srv_cb(msg):
    if msg.action == msg.START:
        rospy.loginfo("START")
    elif msg.action == msg.STOP:
        rospy.loginfo("STOP")
    else:
        rospy.logerr("UNKNOWN AUCTION")
        return False

    return True

def turtle_go(target: list, kp=1, eps=0.1):
    global pub
    road_done = False
    while not road_done:
        dt_x = target[0] - current_pose.x
        dt_y = target[1] - current_pose.y
        dt_z = 0 - current_pose.theta

        if (abs(dt_x) < eps) and (abs(dt_y) < eps):
            road_done = True
            # break
        msg = Twist()
        msg.linear.x = dt_x * kp
        msg.linear.y = dt_y * kp
        msg.angular.z = dt_z * kp
        pub.publish(msg)
        time.sleep(0.1)

def callback_pos(data: Pose):
    global current_pose
    current_pose = data

def listener():
    global current_pose, target

    rospy.Subscriber("/turtle1/pose", Pose, callback_pos)
    eps = 0.1
    kp = 1
    delay = rospy.Rate(10)
    while not rospy.is_shutdown():
        delay.sleep()


if __name__ == '__main__':
    rospy.init_node('control_node')
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    Server(my_param_serverConfig,  reconfigure_cb)
    server = rospy.Service("my_service", my_srv, srv_cb)
    listener()
