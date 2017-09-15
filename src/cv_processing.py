#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from usb_cam_processing.msg import Centroid

def nothing(x):
    pass

class image_converter:

    def __init__(self):
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print("===Camera Manage===", e)


        ###############
        # OpenCV image processing
        ###############
        # _, frame = cv_image.read()

        # cv2.createTrackbar('R',cv_image,0,255,nothing)

        lower_blue = np.array([110,50,50]) # 110, 50, 50
        upper_blue = np.array([130,255,255]) # 130,255,255
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

        # More stuff for finding the shape
        erode = cv2.erode(mask, None, iterations=2)
        dilate = cv2.dilate(erode, None, iterations=2)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        h_max = 0
        x_max = 0
        y_max = 0
        w_max = 0
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if h > h_max:
                h_max = h
                x_max = x
                y_max = y
                w_max = w


        # cv2.rectangle(cv_image, (x_max, y_max), (x_max + w_max, y_max + h_max), [0, 0, 255], 2)

        cv2.circle(cv_image, (x_max + w_max/2, y_max + h_max/2), 7, [255,255,255], -1)

        cv2.imshow("Image window", cv_image)
        cv2.imshow("Res window", dilate)
        cv2.waitKey(1)

        # Publish the position of the ball to a rostopic
        print x_max, y_max
        height, width = cv_image.shape[:2]
        pub = rospy.Publisher('/ball_center', Centroid, queue_size = 10)
        msg = [x_max, y_max, width, height]
        pub.publish(x_max, y_max, width, height)



if __name__ == '__main__':

    rospy.init_node('my_img_processor', anonymous=True)

    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
