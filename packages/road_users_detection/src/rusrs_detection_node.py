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

class UsersDetector(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(UsersDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # publisher (edited image)
        self.detecigm_pub = rospy.Publisher('~detected_usrs/compressed', CompressedImage, queue_size=1)

        # publisher (edited image)
        self.loc_pub = rospy.Publisher('~projected_location', String, queue_size=1)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.camera, queue_size=1)

    def detection(self, image):

        np_arr = np.frombuffer(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        imH, imW = image_np.shape[0:2]

if __name__ == '__main__':
    # create the node
    node = UsersDetector(node_name='rusrs_detection_node')

    # keep spinning
    rospy.spin()