#!/usr/bin/env python3

import rospy
import traceback
from virtual_whiteboard.msg import Marker
import pygame

# get global type parameters from rosparam
screen_w_px = rospy.get_param('/screen_w_px')
screen_h_px = rospy.get_param('/screen_h_px')
screen_w_meters = rospy.get_param('/screen_w_meters')
background = rospy.get_param('/background')
img_path = rospy.get_param('/img_path')

METERS_PER_PIXEL = screen_w_meters / screen_w_px

def listener():

    # initialize this node
    rospy.init_node('whiteboard', anonymous=False)

    pygame.init()

    screen = pygame.display.set_mode((screen_w_px, screen_h_px), pygame.NOFRAME)
    screen.fill((255, 255, 255))

    spray = pygame.image.load(f"{img_path}/spray.png").convert_alpha()

    if background != "":
        bg = pygame.image.load(f"{img_path}/{background}")

        bg_x = (screen_w_px - bg.get_width())/2
        bg_y = (screen_h_px - bg.get_height())/2
        
        screen.blit(bg, (bg_x, bg_y))

    pygame.display.flip()

    # start subscribing to the marker position topic
    # call whiteboard_draw whenever a new Point object is published on the
    # /marker_position topic. Point's z coordinate is ignored
    rospy.Subscriber('/marker_position', Marker, whiteboard_draw, (screen, spray))

    # rospy.spin()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        # if we don't call pygame.event.pump() ocassionally, pygame thinks that 
        # the program has frozen
        pygame.event.pump()
        r.sleep()

def whiteboard_draw(msg_in, params):
    screen, spray = params
    x_coord = msg_in.x / METERS_PER_PIXEL
    y_coord = msg_in.y / METERS_PER_PIXEL
    if msg_in.mode.lower() == "draw":
        pygame.draw.circle(
            screen,
            (0, 0, 0),
            (x_coord, screen_h_px - y_coord), # flip y axis so origin is bottom left
            7
        )
    elif msg_in.mode.lower() == "erase":
        pygame.draw.circle(
            screen,
            (255, 255, 255),
            (x_coord, screen_h_px - y_coord), # flip y axis so origin is bottom left
            40
        )
    elif msg_in.mode.lower() == "spray":
        screen.blit(
            spray,
            (x_coord - 128, screen_h_px - y_coord - 128)
        )
    else:
        rospy.logerr(f"Unknown marker mode {msg_in.mode}")
    pygame.display.flip()
    rospy.loginfo(f"{msg_in.mode} at {x_coord}, {y_coord}")


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        traceback.print_exc()