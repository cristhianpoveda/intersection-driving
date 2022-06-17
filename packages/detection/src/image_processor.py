#!/usr/bin/env python3

import os
import queue
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msg.msgs import Float32MultiArray

class ImgProcessor(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ImgProcessor, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # publisher (model raw features)
        self.flat_buffer = rospy.Publisher('/input_buffer_array', Float32MultiArray, queue_size=1)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.camera, queue_size=1)

    def camera(self, image):
        # import rgb image and resize to desired resolution (160x160)px
        img_arr = np.frombuffer(image.data, np.uint8)
        image_np = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        img_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        width = 160
        height = 160
        dim = (width, height)

        resized = cv2.resize(img_rgb, dim, interpolation = cv2.INTER_AREA)

        # convert from 3 channels rgb to rgb (0xRRGGBB)
        input = [0]

        for ii in range(0, resized.shape[0]):
            for jj in range(0,resized.shape[1]):
                r = img_rgb[ii][jj][0]
                g = img_rgb[ii][jj][1]
                b = img_rgb[ii][jj][2]
                input_value = self.rgb_to_hex(r,g,b)
                int_value = int(input_value, 16)
                float_val = np.float32(int_value)
                input.append(float_val)

        input.pop(0)
        arr = np.array(input)

        # publish model's raw features array
        buf = Float32MultiArray()
        buf.data = arr
        self.flat_buffer.publish(buf.data)


    # convert pixel format to hex rgb (0xRRGGBB)
    def rgb_to_hex(self,r,g,b):
        return ('{:X}{:X}{:X}').format(r, g, b)

if __name__ == '__main__':
    # create the node
    node = ImgProcessor(node_name='image_processor')

    # keep spinning
    rospy.spin()