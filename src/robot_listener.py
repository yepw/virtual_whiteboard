#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from virtual_whiteboard.msg import Marker
from ur5_optimization.msg import EEPoseGoals
from geometry_msgs.msg import Pose
from virtual_whiteboard.srv import Clear, ClearResponse
from omni_msgs.msg import OmniFeedback
import transformations as T
import tf
import numpy as np
import os

distance_to_tv = 0.8

class RobotListener:
    def __init__(self):
        self.task = "train"
        self.task_state = "start"
        self.listener = tf.TransformListener()
        self.marker_position_pub = rospy.Publisher('/marker_position', Marker, queue_size = 1)
        # self.force_feedback_pub = rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size = 1)
        self.clear_board = rospy.ServiceProxy('clear', Clear)
        rospy.Timer(rospy.Duration(0.01), self.timer_cb)
        rospy.Subscriber('study_state', String, self.study_state_cb)

    def study_state_cb(self, msg):
        self.task = msg.data.split('_')[1]
        self.task_state = msg.data.split('_')[2]
        print(self.task)
        if self.task_state == 'start':
            self.clear_board()

    def timer_cb(self, event):

        if self.task_state != 'start':
            return
            
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/tool_tip', rospy.Time(0))
        except:
            return

        # feedback = OmniFeedback()
        # if (trans[0] > 0.8):
        #     feedback.force.z = (trans[1]-0.8) 
        # else:
        #     feedback.force.z = 0.0

        # self.force_feedback_pub.publish(feedback)

        if self.task == "wipe" or self.task == 'write' or self.task == 'spray':
            marker_position = Marker()
            # marker tip not on the whiteboard
            if self.task == "write":
                if (trans[0] < 0.785):
                    return
            elif self.task == "wipe":
                if (trans[0] < 0.78):
                    return

            marker_position.x = -trans[1] + 0.5
            marker_position.y = trans[2] 

            if self.task == "wipe":
                marker_position.radius = 60
                marker_position.mode = "erase"
            elif self.task == "spray":
                marker_position.radius = 40
                marker_position.mode = "spray"
            elif self.task == "write":
                marker_position.radius = 10
                marker_position.mode = "draw"

            self.marker_position_pub.publish(marker_position)

        elif self.task == "water":
            curr_ee_rot = [rot[3], rot[0], rot[1], rot[2]]
            goal_rot_goal = [0.5, 0.5, 0.5, 0.5]
            scaled_axis = T.quaternion_to_scaledAxis(T.quaternion_dispQ(curr_ee_rot, goal_rot_goal))
            rx = scaled_axis[0]
            ry = scaled_axis[1]
            rz = scaled_axis[2]
            angle = np.sqrt(rx**2 + rz**2)
            if angle > 0.17:
                os.system('play -nq -t alsa synth 0.3 sine 440')

        else:
            return


if __name__ == '__main__':
    rospy.init_node("whiteboard_listen_to_robot")
    robotListener =  RobotListener()
    rospy.spin()