#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from virtual_whiteboard.msg import Marker
from geometry_msgs.msg import Pose
from virtual_whiteboard.srv import Clear, ClearResponse
from omni_msgs.msg import OmniButtonEvent
import transformations as T
import tf
import numpy as np
import os

distance_to_tv = 0.8

class RobotListener:
    def __init__(self):
        self.task = "train"
        self.task_state = "start"
        self.tilted_time = 0
        self.listener = tf.TransformListener()
        self.marker_position_pub = rospy.Publisher('whiteboard/draw', Marker, queue_size = 1)

        self.data_log_pub = rospy.Publisher('/data_log', String, queue_size = 1)

        self.clear_board = rospy.ServiceProxy('whiteboard/clear', Clear)
        rospy.Timer(rospy.Duration(0.005), self.timer_cb)
        rospy.Subscriber('study_state', String, self.study_state_cb)
        rospy.Subscriber('user_start',  Bool, self.user_start_cb)
        rospy.Subscriber('/phantom/button', OmniButtonEvent, self.button_cb)
    

    def user_start_cb(self, msg):
        pass

    def study_state_cb(self, msg):
        if len(msg.data.split('_')) >= 2:
            self.task = msg.data.split('_')[1]
            if self.task.split('-')[0] == 'practice':
                self.task = self.task.split('-')[1]
            self.task_state = msg.data.split('_')[2]
            if self.task_state == 'ready':
                self.tilted_time = 0
            print(self.task)

    def button_cb(self, msg):
        if self.task_state != 'running':
            return
        if msg.grey_button == 1 and self.task == 'spray':
            marker_position = Marker()
            marker_position.x = -self.trans[1] + 0.5
            marker_position.y = self.trans[2] 
            marker_position.radius = 40
            marker_position.mode = "spray"
            self.marker_position_pub.publish(marker_position)
            
    def timer_cb(self, event):
        if self.task_state != 'running':
            return
            
        try:
            (self.trans, self.rot) = self.listener.lookupTransform('/base_link', '/tool_tip', rospy.Time(0))
        except:
            return

        if self.task == 'write' or self.task == 'train':
            self.handle_write_task()
        elif self.task == 'spray':
            pass
        elif self.task == "wipe":
            self.handle_wipe_task() 
        elif self.task == "water":
            self.handle_water_task()
        else:
            return

    def handle_write_task(self):
        # marker tip not on the whiteboard
        if (self.trans[0] < 0.785):
            return
        marker_position = Marker()
        marker_position.x = -self.trans[1] + 0.5
        marker_position.y = self.trans[2] 
        marker_position.radius = 10
        marker_position.mode = "draw"
        self.marker_position_pub.publish(marker_position)

    def handle_wipe_task(self):
        # marker tip not on the whiteboard
        if (self.trans[0] < 0.78):
            return
        marker_position = Marker()
        marker_position.x = -self.trans[1] + 0.5
        marker_position.y = self.trans[2] 
        marker_position.radius = 80
        marker_position.mode = "erase"
        self.marker_position_pub.publish(marker_position)

    def handle_water_task(self):
        curr_ee_rot = [self.rot[3], self.rot[0], self.rot[1], self.rot[2]]
        goal_rot_goal = [0.5, 0.5, 0.5, 0.5]
        scaled_axis = T.quaternion_to_scaledAxis(T.quaternion_dispQ(curr_ee_rot, goal_rot_goal))
        rx = scaled_axis[0]
        ry = scaled_axis[1]
        rz = scaled_axis[2]
        angle = np.sqrt(rx**2 + rz**2)
        if angle > 0.17:
            os.system('play -nq -t alsa synth 0.3 sine 440')
            self.tilted_time += 0.3
            self.data_log_pub.publish(String("tilted_time:" + str(self.tilted_time)))

if __name__ == '__main__':
    rospy.init_node("whiteboard_listen_to_robot")
    robotListener =  RobotListener()
    rospy.spin()