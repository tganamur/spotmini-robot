#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from std_msgs.msg import Float64 


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
    def __init__(self):
        rospy.init_node('perception')

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        #self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        # self.center_point_pub = rospy.Publisher("~center_point_image", Point, queue_size=10)
        self.height_pub = rospy.Publisher('/spotmini/input', Float64, queue_size = 10)
        #self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=10)

        self.width = None
        self.height = None
        
        self.lower_hsv = np.array([113, 123, 90])
        self.upper_hsv = np.array([160, 255, 255])
        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message (look this message type up online)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

        self.width = msg.width
        self.height = msg.height

    def pixel_to_point(self, u, v, depth):
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = ((u-self.width/2) * depth)/self.fx
        Y = ((v-self.width/2) * depth)/self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()
                # print('processing image')

        except Exception as e:
            print("Error:", e)
            self.process_images()
            # print('processing image')

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def process_images(self):
        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)
        # TODO: Define range for cup color in HSV
        # NOTE: You can visualize how this is performing by viewing the result of the segmentation in rviz
        # To see the current HSV values in the center row of the image (where your cup should be), we will print out
        # the HSV mean of the HSV values of the center row. You should add at least +/- 10 to the current values to define your range.
        # mean_center_row_hsv_val = np.mean(hsv[len(hsv)//2], axis=0)
        #print("Current mean values at center row of image: ", mean_center_row_hsv_val)
       

        # TODO: Threshold the image to get only cup colors
        # HINT: Lookup cv2.inRange() or np.where()
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # TODO: Get the coordinates of the cup points on the mask
        # HINT: Lookup np.nonzero()
        y_coords, _ = np.nonzero(mask)

        # If there are no detected points, exit
        if len(y_coords) == 0:
            # print("No points detected. Is your color filter wrong?")
            return

        # Calculate the center of the detected region by 
        # center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))
        # Publish the center point of detected object in image plane
        
        height_robot = 0

        # threshold to start changing height 
        # TODO: figure out units from center_point
        threshold = 15 
        height_limit = 50
        
        cond = center_y - (self.height/2) 
        if cond > threshold or cond < -threshold:
            height_robot = cond
        else:
            height_robot = 0
            
        height_robot = max(-height_limit, min(height_limit, height_robot))
        
        rospy.loginfo('Current center_y: %s, height_robot %s, cond: %s', str(center_y), str(height_robot), str(cond))
        self.height_pub.publish(height_robot)
        
        # self.center_point_pub.publish(Point(center_x, center_y, 0))
        # rate = rospy.Rate(10)
        # rate.sleep()
        return 
'''
        # Fetch the depth value at the center
        depth = self.cv_depth_image[center_y, center_x]

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
            camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000

            

            # Overlay cup points on color image for visualization
            cup_img = self.cv_color_image.copy()
            cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
            cv2.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)  # Draw green circle at center
            
            # Convert to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
            self.image_pub.publish(ros_image)
'''
if __name__ == '__main__':
    ObjectDetector()
