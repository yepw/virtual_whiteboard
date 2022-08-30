#!/usr/bin/env python3
"""
This file is just used to test out the virtual_whiteboard
node by spoofing a moving marker position
"""

import rospy
import traceback
from geometry_msgs.msg import Point
# get global type parameters from rosparam
screen_w_px = rospy.get_param('/screen_w_px')
screen_h_px = rospy.get_param('/screen_h_px')
screen_w_meters = rospy.get_param('/screen_w_meters')

def main():
    rospy.init_node('spoof_marker', anonymous=False)

    marker_pub = rospy.Publisher("/marker_position", Point, queue_size=1)

    marker_pos = Point()
    marker_pos.x = 0
    marker_pos.y = 0.2
    marker_pos.z = 0

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        marker_pos.x += 0.001
        marker_pos.y += 0
        marker_pub.publish(marker_pos)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        traceback.print_exc()