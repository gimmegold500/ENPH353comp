#!/usr/bin/env python
import cv2 as cv
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

import numpy as np

class license_plate_detector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.template_path = "INSERT_BLANK_IMAGE_LINK"
        lh = 116
        ls = 40
        lv = 41
        uh = 124
        us = 255
        uv = 255
        self.lower_hsv_b = np.array([lh,ls,lv])
        self.upper_hsv_b = np.array([uh,us,uv])

        lh = 0
        ls = 0
        lv = 112
        uh = 17
        us = 0
        uv = 135
        self.lower_hsv_w = np.array([lh,ls,lv])
        self.upper_hsv_w = np.array([uh,us,uv])
    
    def callback(self, data):
        time.sleep(0.5)
        
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height = img.shape[0]
        img = img[height // 2 :  (3 * height) // 4]
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask_blue = cv.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) // 255
        mask_white = cv.inRange(hsv, self.lower_hsv_w, self.upper_hsv_w) // 255

        print(np.sum(mask_white))
        print(np.sum(mask_blue))
        
        if (np.sum(mask_blue) >25000 and np.sum(mask_white) > 450):
            print("CAR!")

def main(args):
    lpd = license_plate_detector()
    rospy.init_node('license_plate_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)