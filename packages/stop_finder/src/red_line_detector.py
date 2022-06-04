#!/usr/bin/env python3

import os
import queue
import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String

class RedLineDetector(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RedLineDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # publisher (edited image)
        self.center_pub = rospy.Publisher('~detected_center/compressed', CompressedImage, queue_size=1)

        # publisher (edited image)
        self.dist_pub = rospy.Publisher('~stop_line_distance', String, queue_size=1)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.camera, queue_size=1)

    def camera(self, image):

        np_arr = np.frombuffer(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # remove the image undesired section
        height, width = image_np.shape[0:2]
        image_np = image_np[int(height/2) :, int(width/4):int(width*0.75), :]

        # hsv image
        imghsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

        # HSV red color range
        red_lower_1 = np.array([0, 100, 20])
        red_upper_1 = np.array([10, 255, 255])
        red_lower_2 = np.array([160,100,20])
        red_upper_2 = np.array([179,255,255])

        mask1 = cv2.inRange(imghsv, red_lower_1, red_upper_1)
        mask2 = cv2.inRange(imghsv, red_lower_2, red_upper_2)
 
        red_mask = mask1 + mask2

        contours, hierachy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)

            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            cv2.drawContours(image_np, contours, -1, (0,255,0), 2)
        else:
            cX, cY = 0, 0
            
        image_center = cv2.circle(image_np, (cX, cY), 3, (255, 0, 0), -1)
        dist = -0.00001655*(cY**3)+0.005856*(cY**2)-0.7631*cY+51.64

        # Create a compressed image message
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_center)[1]).tostring()

        # Publish new image
        self.center_pub.publish(msg)

        dist_msg = str(dist)
        # Publish new image
        self.dist_pub.publish(dist_msg)  

if __name__ == '__main__':
    # create the node
    node = RedLineDetector(node_name='red_line_detector')

    # keep spinning
    rospy.spin()