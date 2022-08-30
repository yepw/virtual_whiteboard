#!/usr/bin/env python3

from math import sqrt
import rospy
import traceback
from virtual_whiteboard.msg import Marker
from virtual_whiteboard.srv import Clear, ClearResponse
from std_msgs.msg import String
from std_msgs.msg import Bool
import pygame

# get global type parameters from rosparam
screen_w_px = rospy.get_param('/screen_w_px')
screen_h_px = rospy.get_param('/screen_h_px')
screen_w_meters = rospy.get_param('/screen_w_meters')
background = rospy.get_param('/background')
img_path = rospy.get_param('/img_path')
endpoint = tuple(rospy.get_param('/endpoints')[background])
endpoint_radius = rospy.get_param('/endpoint_radius')[background]

METERS_PER_PIXEL = screen_w_meters / screen_w_px

task_status = "not_started"

def main():
    global task_status

    rospy.init_node('whiteboard', anonymous=False)

    # create a publisher for the status of the whiteboard tracing task
    # will publish "not started", "started", or "finished" when status changes
    status_pub = rospy.Publisher('/whiteboard_status', String, queue_size=1)

    # give the publisher time to boot up
    rospy.sleep(3)

    status_pub.publish(String(task_status))

    pygame.init()

    screen = pygame.display.set_mode((screen_w_px, screen_h_px), pygame.NOFRAME, display=rospy.get_param('/display_num'))
    screen.fill((255, 255, 255))

    spray = pygame.image.load(f"{img_path}/spray.png").convert_alpha()

    draw_background(screen, background)

    # for debugging - draw the end point and its radius as a red circle
    pygame.draw.circle(
        screen,
        (255, 0, 0),
        (endpoint[0], screen_h_px - endpoint[1]),
        endpoint_radius,
    )

    pygame.display.flip()

    # start subscribing to the marker position topic
    # call whiteboard_draw whenever a new Marker msg is published on the
    # /marker_position topic.
    rospy.Subscriber('/marker_position', Marker, whiteboard_draw, (screen, spray, endpoint, endpoint_radius, status_pub))
    
    # create a service that clears the board
    # the lambda function is a workaround to pass extra parameters to the handler
    # https://answers.ros.org/question/247540/pass-parameters-to-a-service-handler-in-rospy/
    rospy.Service("clear", Clear, lambda msg: handle_clear_srv(msg, (screen, background)))

    # rospy.spin()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        # if we don't call pygame.event.pump() ocassionally, pygame thinks that 
        # the program has frozen
        pygame.event.pump()
        r.sleep()

def whiteboard_draw(msg_in, params):
    global task_status

    screen, spray, endpoint, endpoint_radius, status_pub = params
    x_coord = msg_in.x / METERS_PER_PIXEL
    y_coord = msg_in.y / METERS_PER_PIXEL

    if msg_in.mode.lower() == "draw":
        pygame.draw.circle(
            screen,
            tuple(rospy.get_param('/draw_color')),
            (x_coord, screen_h_px - y_coord), # flip y axis so origin is bottom left
            rospy.get_param('/draw_radius'),
        )
    elif msg_in.mode.lower() == "erase":
        pygame.draw.circle(
            screen,
            (255, 255, 255),
            (x_coord, screen_h_px - y_coord), # flip y axis so origin is bottom left
            rospy.get_param('/erase_radius'),
        )
    elif msg_in.mode.lower() == "spray":
        screen.blit(
            spray,
            (x_coord - 128, screen_h_px - y_coord - 128)
        )
    else:
        rospy.logerr(f"Unknown marker mode {msg_in.mode}")
    pygame.display.flip()
    # rospy.loginfo(f"{msg_in.mode} at {x_coord}, {y_coord}")

    # check if the task is done by checking how close it is to the end point
    if ((sqrt((x_coord-endpoint[0])**2 + (y_coord-endpoint[1])**2) < endpoint_radius) and
    task_status != "finished"):
        task_status = "finished"
        status_pub.publish(String(task_status))
    elif task_status != "started" and task_status != "finished":
        task_status = "started"
        status_pub.publish(String(task_status))

def handle_clear_srv(msg, params):
    global task_status
    screen, background = params
    print("cleared")
    screen.fill((255, 255, 255))
    draw_background(screen, background)
    task_status = "not started"
    return ClearResponse(True)

def draw_background(screen, background):
    if background != "":
        bg = pygame.image.load(f"{img_path}/{background}")

        bg_x = (screen_w_px - bg.get_width())/2
        bg_y = (screen_h_px - bg.get_height())/2
        
        screen.blit(bg, (bg_x, bg_y))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        traceback.print_exc()