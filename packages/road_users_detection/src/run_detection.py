#!/usr/bin/env python3

import os
import queue
import numpy as np
import rospy
import rospkg
import math
import cv2
import tflite_runtime.interpreter as tflite
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String

class UsrsDetector(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(UsrsDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # load file paths
        package = rospkg.RosPack()

        MODEL_PATH = package.get_path('road_users_detection') + '/TFLite_model/best-int8.tflite'
        self.LABELS_PATH = package.get_path('road_users_detection') + '/TFLite_model/labelmap.txt'

        with open(self.LABELS_PATH, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # initialize tensorflow interpreter

        self.interpreter = tflite.Interpreter(model_path=MODEL_PATH)
        self.interpreter.allocate_tensors()

        # save input and output details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # get expected model's input size
        self.modelH = self.input_details[0]['shape'][1]
        self.modelW = self.input_details[0]['shape'][2]

        # publisher (edited image)
        self.detect_pub = rospy.Publisher('~detected_usrs/compressed', CompressedImage, queue_size=1)

        # publisher (edited image)
        self.location_pub = rospy.Publisher('~usrs_location', String, queue_size=1)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.detection, queue_size=1)

    def detection(self, image):

        np_arr = np.frombuffer(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        imH, imW = image_np.shape[0:2]

        imgrgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        imgrsz = cv2.resize(imgrgb, (self.modelW, self.modelH))
        model_input = np.expand_dims(imgrsz, axis=0)

        # set tensor
        self.interpreter.set_tensor(self.input_details[0]['index'], model_input)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[3]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        #output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        #results = np.squeeze(output_data)

        #top_k = results.argsort()[-5:][::-1]
        #label1 = load_labels(self.LABELS_PATH)
        #for i in top_k:
        #    self.
        #    print('{:08.6f}: {}'.format(float(results[i] / 255.0), label1[i]))

        for i in range(len(scores)):
            if ((scores[i] > 0.2) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
            
                detectedimg = cv2.rectangle(image_np, (xmin,ymin), (xmax,ymax), (210, 210, 70), 2)

                # Draw label
                object_name = self.labels[int(classes[i])] 
                label = '%s: %d%%' % (object_name, int(scores[i]*100))
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                label_ymin = max(ymin, labelSize[1] + 10)
                detectedimg = cv2.rectangle(image_np, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED)
                detectedimg = cv2.putText(image_np, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        # Create a compressed image message with detection results
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', detectedimg)[1]).tostring()

        # Publish new image
        self.detect_pub.publish(msg)

if __name__ == '__main__':
    # create the node
    node = UsrsDetector(node_name='run_detection')

    # keep spinning
    rospy.spin()