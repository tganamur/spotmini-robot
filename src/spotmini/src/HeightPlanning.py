#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo 
from std_msgs.msg import Float64 

center_y = 0
img_height = 0
def point_callback(msg):
    # get center_y coord of cup in image frame from Point.msg
    global center_y 
    center_y = msg.y

def camera_callback(msg):
    # get the height of the image plane from camera info
    global img_height 
    img_height = msg.height

def height_planning():
    rospy.init_node("planning")

    # subscribe to center_point topic to get center_point coord
    rospy.Subscriber('/perception/center_point_image', Point, point_callback)
    # subscribe to camera_info topic to get camera instric info 
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_callback)

    # setup publisher for publishing to height of robot topic 
    height_pub = rospy.Publisher('/spotmini/input', Float64, queue_size = 10)

    rate = rospy.Rate(10)
    
    # intial height 
    height_robot = 0

    # threshold to start changing height 
    # TODO: figure out units from center_point
    threshold = 15 
    height_limit = 50

    # continously publish to height_robot topic (4 Hz)
    while not rospy.is_shutdown():
        # figure out how far from image plane center the center_y is 
        cond = center_y - (img_height/2) 
        if cond > threshold or cond < -threshold:
            height_robot = cond
        else:
            height_robot = 0
            
        if height_robot > height_limit:
            height_robot = height_limit
        elif height_robot < -height_limit:
            height_robot = -height_limit
       

        rospy.loginfo('height_robot: ' + str(height_robot))
        rospy.loginfo('cond: ' + str(cond))
        if not(cond > threshold or cond < -threshold):
            height_pub.publish(height_robot)

    rospy.spin()


if __name__ == '__main__':
    height_planning()
