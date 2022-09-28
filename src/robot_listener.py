#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from virtual_whiteboard.msg import Marker
from ur5_optimization.msg import EEPoseGoals
from geometry_msgs.msg import Pose
import tf

distance_to_tv = 0.8

class RobotListener:
    def __init__(self):
        self.task = "training"
        self.listener = tf.TransformListener()
        self.marker_position_pub = rospy.Publisher('/marker_position', Marker, queue_size = 1)
        rospy.Subscriber('tool_tip_pose', Pose, self.tool_tip_pose_cb)
        rospy.Subscriber('study_state', String, self.study_state_cb)

    def study_state_cb(self, msg):
        self.task = msg.data

    def tool_tip_pose_cb(self, msg):
        marker_position = Marker()

        # marker tip not on the whiteboard
        if (msg.position.x < 0.78):
            return

        marker_position.x = -msg.position.y + 0.5
        marker_position.y = msg.position.z 

        if self.task == "wiping":
            marker_position.radius = 60
            marker_position.mode = "erase"
        elif self.task == "spraying":
            marker_position.radius = 30
            marker_position.mode = "spray"
        elif self.task == "writing":
            marker_position.radius = 10
            marker_position.mode = "draw"
        else:
            return

        self.marker_position_pub.publish(marker_position)


if __name__ == '__main__':
    rospy.init_node("whiteboard_listen_to_robot")
    robotListener =  RobotListener()
    rospy.spin()