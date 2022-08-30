#!/usr/bin/env python3
"""
This file is just used to test out the virtual_whiteboard
node by spoofing a moving marker position
"""

import rospy
import traceback
from virtual_whiteboard.msg import Marker
# get global type parameters from rosparam
screen_w_px = rospy.get_param('/screen_w_px')
screen_h_px = rospy.get_param('/screen_h_px')
screen_w_meters = rospy.get_param('/screen_w_meters')

def main():
    rospy.init_node('spoof_marker', anonymous=False)

    marker_pub = rospy.Publisher("/marker_position", Marker, queue_size=1)

    modes = ["draw", "spray", "erase"]

    marker_pos = Marker()
    marker_pos.x = 0
    marker_pos.y = 0.2
    marker_pos.mode = modes[0]

    r = rospy.Rate(100)
    i = 0
    while not rospy.is_shutdown():
        marker_pos.mode = modes[i%3]
        if marker_pos.x > screen_w_meters:
            marker_pos.x = 0
            i += 1
        marker_pos.x += 0.001
        marker_pos.y += 0
        marker_pub.publish(marker_pos)
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        traceback.print_exc()