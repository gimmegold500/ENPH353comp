#!/usr/bin/env python
import cv2 as cv
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import os

import numpy as np

class license_plate_detector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.template_path = "INSERT_BLANK_IMAGE_LINK"

        # blue
        lh = 116
        ls = 40
        lv = 41
        uh = 124
        us = 255
        uv = 255
        self.lower_hsv_b = np.array([lh,ls,lv])
        self.upper_hsv_b = np.array([uh,us,uv])

        # dark thresholds
        lh = 0
        ls = 0
        lv = 94
        uh = 0
        us = 0
        uv = 140
        self.lower_hsv_w_dark = np.array([lh,ls,lv])
        self.upper_hsv_w_dark = np.array([uh,us,uv])

        # light thresholds
        lh = 90
        ls = 0
        lv = 74
        uh = 123
        us = 47
        uv = 96
        self.lower_hsv_plate_dark = np.array([lh, ls, lv])
        self.upper_hsv_plate_dark = np.array([uh, us, uv])

        self.lower_hsv_w_bright = np.array([0, 0, 191])
        self.upper_hsv_w_bright = np.array([0, 0, 206])
        self.lower_hsv_plate_bright = np.array([103, 0, 112])
        self.upper_hsv_plate_bright = np.array([155, 35, 184])

        self.imNum = 0
    
    def callback(self, data):
        time.sleep(0.2)
        
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height = img.shape[0]
        width = img.shape[1]
        img = img[height // 2 :  (3 * height) // 4]
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # get masks
        mask_blue = cv.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) // 255
        mask_white_dark = cv.inRange(hsv, self.lower_hsv_w_dark, self.upper_hsv_w_dark) // 255
        mask_white_bright = cv.inRange(hsv, self.lower_hsv_w_bright, self.upper_hsv_w_bright) // 255
        mask_plate_dark = cv.inRange(hsv, self.lower_hsv_plate_dark, self.upper_hsv_plate_dark) // 255
        mask_plate_bright = cv.inRange(hsv, self.lower_hsv_plate_bright, self.upper_hsv_plate_bright) // 255


        # erode and dilate images
        kernel_3 = np.ones((3, 3), np.uint8)
        kernel_5 = np.ones((5, 5), np.uint8)

        # slice images in half
        mask_white_dark = cleanImage(mask_white_dark, kernel_5)
        mask_white_bright = cleanImage(mask_white_bright, kernel_5)
        mask_plate_dark = cleanImage(mask_plate_dark, kernel_3)
        mask_plate_bright = cleanImage(mask_plate_bright, kernel_3)

        mask_blue_l = mask_blue[:,0:width // 2]
        mask_white_bright_l = mask_white_bright[:,0:width // 2]
        mask_white_dark_l = mask_white_dark[:, 0:width//2]
        mask_plate_bright_l = mask_plate_bright[:, 0:width//2]
        mask_plate_dark_l = mask_plate_dark[:, 0:width//2]

        # process car if needed
        if car_is_spotted(self, mask_blue_l, mask_white_bright_l, mask_plate_bright_l):
            process_car(self, mask_blue_l, mask_white_bright_l, mask_plate_bright_l, img[:, 0:width//2], kernel_3)
        
        if car_is_spotted(self, mask_blue_l, mask_white_dark_l, mask_plate_dark_l):
            process_car(self, mask_blue_l, mask_white_dark_l, mask_plate_dark_l, img[:, 0:width//2], kernel_3)

def car_is_spotted(self, blue_vals, white_vals, grey_vals):
    print(np.sum(grey_vals))
    return np.sum(blue_vals) > 25000 and np.sum(blue_vals) < 37500 and np.sum(white_vals) > 500

def process_car(self, blue_vals, white_vals, grey_vals, og_img, kernel):
    print("CAR!")

    shape = np.shape(blue_vals)

    mask_blue = np.reshape(blue_vals, (shape[0], shape[1], -1))
    mask_grey = og_img * mask_blue

    shape = np.shape(white_vals)
    mask_white = np.reshape(white_vals, (shape[0], shape[1], -1))
    mask_grey = og_img * mask_white

    shape = np.shape(grey_vals)
    grey_vals = np.reshape(grey_vals, (shape[0], shape[1], -1))
    plate = og_img * grey_vals

    plate = cleanImage(plate, kernel)

    plate = plate + mask_grey

    plate = cv.cvtColor(plate, cv.COLOR_HSV2BGR)
    plate = cv.cvtColor(plate, cv.COLOR_BGR2GRAY)

    kernel_5 = np.ones((20, 20), np.uint8)
    plate = cv.dilate(plate, kernel_5, iterations = 1)
    plate = cv.erode(plate, kernel_5, iterations = 1)
    
    plate = cleanImage(plate, kernel_5)

    im2, contours, hierarchy = cv.findContours(plate, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = get_list_contours(contours)
    new_contours = [order_points(contours).astype(int)]
    print(new_contours)
    # cv.drawContours(og_img, new_contours, -1, (0, 255, 0), 3)

    cv.waitKey(2)

    new_plate = four_point_transform(self, og_img, contours)
    cv.imshow("warped plate", new_plate)
    cv.waitKey(2)
    cv.imwrite(os.path.dirname(os.path.realpath(__file__)) + '/plate' + str(self.imNum) + '.png', new_plate)
    self.imNum += 1

def cleanImage(image, kernel):
    image = cv.erode(image, kernel, iterations=1)
    return cv.dilate(image, kernel, iterations=1)

def order_points(pts):
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    # pts = np.reshape(pts, (-1, 2))
    rect = np.zeros((4, 2), dtype = "float32")
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = np.sum(pts, axis = 2)

    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis = 2)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # return the ordered coordinates

    rect = np.array(rect)
    print(rect)
    return rect

def get_list_contours(contours):
    retList = []

    for contour_list in contours:
        for i in range(np.shape(contour_list)[0]):
            retList.append(contour_list[i])
    
    retList = np.array(retList)
    return retList

def four_point_transform(self, image, pts):
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
	dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
	# compute the perspective transform matrix and then apply it
	M = cv.getPerspectiveTransform(rect, dst)
	warped = cv.warpPerspective(image, M, (maxWidth, maxHeight))
	# return the warped image
	return warped

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