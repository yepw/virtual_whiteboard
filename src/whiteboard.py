#!/usr/bin/env python3

from math import sqrt
from tkinter import S

import rospy
import rospkg
import traceback
from virtual_whiteboard.msg import Marker
from virtual_whiteboard.srv import Clear, ClearResponse
from std_msgs.msg import String
from std_msgs.msg import Bool, Int64
import pygame
import numpy as np
import random
from subprocess import call

# get global type parameters from rosparam
screen_w_px = rospy.get_param('/screen_w_px')
screen_h_px = rospy.get_param('/screen_h_px')
screen_w_meters = rospy.get_param('/screen_w_meters')
img_path = rospkg.RosPack().get_path('virtual_whiteboard') + "/img"

METERS_PER_PIXEL = screen_w_meters / screen_w_px

coverage_threshold = 1000

class Whiteboard:
    def __init__(self):
        # set system volumn
        call(["amixer", "-D", "pulse", "sset", "Master", "35%"])

        global task_status

        # create a publisher for the status of the whiteboard tracing task
        # will publish "not started", "started", or "finished" when status changes
        self.user_finish_pub = rospy.Publisher('/user_finish', Bool, queue_size = 1)
        self.data_log_pub = rospy.Publisher('/data_log', String, queue_size = 1)

        self.pen_points = ""

        # give the publisher time to boot up
        rospy.sleep(1)

        pygame.init()

        self.font = pygame.font.Font('freesansbold.ttf', 32)

        self.screen = pygame.display.set_mode((screen_w_px, screen_h_px), pygame.NOFRAME, display=rospy.get_param('/display_num'))
        self.screen.fill((255, 255, 255))

        self.spray = pygame.image.load(f"{img_path}/spray.png").convert_alpha()

        background = rospy.get_param('/background')
        self.draw_background(background)
        
        pygame.display.flip()

        # start subscribing to the marker position topic
        # call whiteboard_draw whenever a new Marker msg is published on the
        # /marker_position topic.
        rospy.Subscriber('whiteboard/draw', Marker, self.whiteboard_draw)
        rospy.Subscriber('whiteboard/background', String, self.change_background)

        # create a service that clears the board
        # the lambda function is a workaround to pass extra parameters to the handler
        # https://answers.ros.org/question/247540/pass-parameters-to-a-service-handler-in-rospy/
        rospy.Service("whiteboard/clear", Clear, lambda msg: self.handle_clear_srv)

        rospy.Subscriber('whiteboard/countdown', String, self.update_countdown)

        self.counter = 0

        rospy.Timer(rospy.Duration(0.01), self.timer_cb) 

    def timer_cb(self, event):
        pygame.event.pump()
        pygame.display.flip()

    def update_countdown(self, msg):
        # text = self.font.render("  Time remaining  {} s   ".format(msg.data), True, (0,0,0), (255,255,255))
        text = self.font.render(msg.data, True, (0,0,0), (255,255,255))
        textRect = text.get_rect()
        textRect.center = (screen_w_px // 2, screen_h_px // 15)
        self.screen.blit(text, textRect)

    def whiteboard_draw(self, msg_in):
        global task_status

        x_coord = msg_in.x / METERS_PER_PIXEL
        y_coord = msg_in.y / METERS_PER_PIXEL

        if msg_in.mode.lower() == "draw":
            pygame.draw.circle(
                self.screen,
                tuple(rospy.get_param('/draw_color')),
                (x_coord, screen_h_px - y_coord), # flip y axis so origin is bottom left
                msg_in.radius
            )
            s = "[" + str(x_coord) + "," + str(screen_h_px - y_coord) + "]"
            self.data_log_pub.publish(String("pen_points:" + s))
        elif msg_in.mode.lower() == "erase":
            pygame.draw.circle(
                self.screen,
                (255, 255, 255),
                (x_coord, screen_h_px - y_coord), # flip y axis so origin is bottom left
                msg_in.radius
            )
            s = "[" + str(x_coord) + "," + str(screen_h_px - y_coord) + "]"
            # self.data_log_pub.publish(String("pen_points:" + s))
            # self.counter += 1
            # if self.counter >= 10:
            self.data_log_pub.publish(String("unwiped_area:{}".format(self.get_non_white_area())))
            #     self.counter = 0
        elif msg_in.mode.lower() == "spray":
            spray = pygame.transform.scale(self.spray, (msg_in.radius*2, msg_in.radius*2))
            for i in range(10):
                self.screen.blit(
                    spray,
                    (x_coord - msg_in.radius + random.uniform(-10, 10), screen_h_px - y_coord - msg_in.radius + random.uniform(-10, 10))
                )
        else:
            rospy.logerr(f"Unknown marker mode {msg_in.mode}")
        # rospy.loginfo(f"{msg_in.mode} at {x_coord}, {y_coord}")

        # check if the task is done
        return

    def change_background(self, msg):
        if len(msg.data.split(':')) == 2:
            file_name = msg.data.split(":")[0]
            ratio = float(msg.data.split(":")[1])
            self.draw_background(file_name, ratio)
        else:
            self.draw_background(msg.data)

    def handle_clear_srv(self, msg):
        global task_status
        print("cleared")
        self.screen.fill((255, 255, 255))
        self.draw_background(self.background)
        task_status = "not started"
        pygame.display.flip()
        return ClearResponse(True)

    def draw_background(self, background, ratio = 1.0):
        self.background = background
        bg = pygame.image.load(f"{img_path}/{background}")
        scaled_bg = pygame.transform.scale(bg, (int(bg.get_width()*ratio), int(bg.get_height()*ratio)))
        bg_x = (screen_w_px - bg.get_width() * ratio) / 2
        bg_y = (screen_h_px - bg.get_height() * ratio) / 2
        
        self.screen.fill((255, 255, 255))
        self.screen.blit(scaled_bg, (bg_x, bg_y))

        # if background == "ros_curve.svg" or background == "hri_curve.svg" \
        #     or background == "wipe.svg":
        #     pygame.display.flip()
        #     pa = pygame.PixelArray(self.screen)
        #     non_white_index = np.where(np.array(pa) != int("FFFFFF", 16) )
        #     self.data_log_pub.publish(
        #             String("background:{}".format(
        #                 str(non_white_index[0].tolist()) + 
        #                 "," + str(non_white_index[1].tolist()))))

        # if background == "hri_curve.svg" or background == "ros_curve.svg" or background == "lab_curve.svg":
        #     self.terminate_condition = "distance"
        #     self.endpoint = tuple(rospy.get_param('/endpoints')[background])
        #     self.endpoint_radius = rospy.get_param('/endpoint_radius')[background]

            # for debugging - draw the end point and its radius as a red circle
            # pygame.draw.circle(
            #     self.screen,
            #     (255, 0, 0),
            #     (self.endpoint[0], screen_h_px - self.endpoint[1]),
            #     self.endpoint_radius,
            # )
        # if background == "wipe.svg":
        #     self.terminate_condition = "cover"
        # elif background == "spray.svg":
        #     self.terminate_condition = "in_four_regions"
        #     self.sprayed_regions = [False, False, False, False]
        # else:
        #     self.terminate_condition = None
        if background == "wipe.svg":
            self.data_log_pub.publish(String("unwiped_area:{}".format(self.get_non_white_area())))

    def get_non_white_area(self):
        # get the number of pixels that are not white
        pa = pygame.PixelArray(self.screen)
        a = np.array(pa[:, 100:])
        non_white_counts = np.count_nonzero(a != int("FFFFFF", 16) )
        pa.close()
        # print("non_white_counts", non_white_counts)
        return non_white_counts

if __name__ == '__main__':

    rospy.init_node('whiteboard', anonymous=False)
    wb = Whiteboard()
    rospy.spin()