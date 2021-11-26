#!/usr/bin/env python
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import os
import csv

import numpy as np

class license_plate_detector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()

        # blue
        lh, ls, lv= 116, 40, 41
        uh, us, uv = 124, 255, 255
        self.lower_hsv_b = np.array([lh,ls,lv])
        self.upper_hsv_b = np.array([uh,us,uv])

        # dark thresholds
        lh, ls, lv = 0, 0, 94
        uh, us, uv = 0, 0, 140
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
        self.lower_hsv_plate3 = np.array([103, 0, 80])
        self.upper_hsv_plate3 = np.array([135, 41, 180])

        self.imNum = 0

        # with open('~/ros_ws/src/enph353/enph353_gazebo/scripts/plates.csv') as csvfile:
        #     reader = csv.reader(csvfile)

        #     rows = []

        #     for row in reader:
        #         rows.append(row)
            
        #     print(rows)
        
        # csvfile.close()

        self.letters = [['M', 'B'], ['S', 'S'], ['M', 'Y'], ['N', 'J'], ['T', 'N'], ['O', 'R'], ['W', 'P'], ['L', 'Z']]
        self.numbers = [[3, 1], [3, 7], [0, 4], [2, 9], [1, 3], [4, 7], [7, 1], [7, 1]]
    
    def callback(self, data):
        time.sleep(0.2)
        
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height = img.shape[0]
        width = img.shape[1]
        img = img[height // 2 :  (4 * height) // 5]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # get masks
        mask_blue = cv2.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) // 255
        mask_white_dark = cv2.inRange(hsv, self.lower_hsv_w_dark, self.upper_hsv_w_dark) // 255
        mask_white_bright = cv2.inRange(hsv, self.lower_hsv_w_bright, self.upper_hsv_w_bright) // 255
        mask_plate_dark = cv2.inRange(hsv, self.lower_hsv_plate_dark, self.upper_hsv_plate_dark) // 255
        mask_plate_bright = cv2.inRange(hsv, self.lower_hsv_plate_bright, self.upper_hsv_plate_bright) // 255
        mask_plate_3 = cv2.inRange(hsv, self.lower_hsv_plate3, self.upper_hsv_plate3) // 255

        # erode and dilate images
        kernel_3 = np.ones((3, 3), np.uint8)
        kernel_5 = np.ones((5, 5), np.uint8)

        # slice images in half
        mask_white_dark = cleanImage(mask_white_dark, kernel_5)
        mask_white_bright = cleanImage(mask_white_bright, kernel_5)
        mask_plate_dark = cleanImage(mask_plate_dark, kernel_3)
        mask_plate_bright = cleanImage(mask_plate_bright, kernel_3)
        mask_plate_3 = cleanImage(mask_plate_3, kernel_3)

        mask_blue_l = mask_blue[:,0:width // 2]

        mask_white_bright_l = mask_white_bright[:,0:width // 2]

        mask_white_dark_l = mask_white_dark[:, 0:width//2]
        mask_plate_bright_l = mask_plate_bright[:, 0:width//2] + mask_plate_3[:, 0: width //2]

        mask_plate_dark_l = mask_plate_dark[:, 0:width//2]
        mask_plate_dark_l = mask_plate_dark[:, 0:width//2] + mask_plate_3[:, 0: width //2]

        mask_blue_r = mask_blue[:,width // 2:]

        mask_white_bright_r = mask_white_bright[:,width // 2:]
        mask_white_dark_r = mask_white_dark[:,width // 2:]
        mask_plate_bright_r = mask_plate_bright[:,width // 2:]
        mask_plate_dark_r = mask_plate_dark[:,width // 2:]

        # process car if needed
        if car_is_spotted(self, mask_blue_l, mask_white_bright_l, mask_plate_bright_l):
            process_car(self, mask_blue_l, mask_white_bright_l, mask_plate_bright_l, img[:, 0:width//2], kernel_3)
        
        if car_is_spotted(self, mask_blue_l, mask_white_dark_l, mask_plate_dark_l):
            process_car(self, mask_blue_l, mask_white_dark_l, mask_plate_dark_l, img[:, 0:width//2], kernel_3)
        
        if car_is_spotted(self, mask_blue_r, mask_white_dark_r, mask_plate_dark_r):
            process_car(self, mask_blue_r, mask_white_dark_r, mask_plate_dark_r, img[:, width//2:], kernel_3)

        if car_is_spotted(self, mask_blue_r, mask_white_bright_r, mask_plate_bright_r):
            process_car(self, mask_blue_r, mask_white_bright_r, mask_plate_bright_r, img[:, 0:width//2], kernel_3)

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
    cv2.imshow("plate", plate)

    plate = cleanImage(plate, kernel)
    plate = plate + mask_grey

    plate = cv2.cvtColor(plate, cv2.COLOR_HSV2BGR)
    plate = cv2.cvtColor(plate, cv2.COLOR_BGR2GRAY)

    kernel_5 = np.ones((20, 20), np.uint8)
    plate = cv2.dilate(plate, kernel_5, iterations = 1)
    plate = cv2.erode(plate, kernel_5, iterations = 1)
    plate = cleanImage(plate, kernel_5)

    im2, contours, hierarchy = cv2.findContours(plate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if (len(contours) > 0):
        contours = get_list_contours(contours)
        cv2.imshow("contours", og_img)
        cv2.waitKey(2)

        new_contours = [order_points(contours).astype(int)]
        new_plate = four_point_transform(self, og_img, contours)
        
        cv2.imshow("warped plate", new_plate)
        cv2.waitKey(2)

        savePlate(self, new_plate)

def savePlate(self, plate):
    parkingSpot = plate[66 : 130, 50 :]
        
    cv2.imshow("parking", parkingSpot)
    cv2.waitKey(2)

    actual_plate = plate[140 : 175, :]
    letters = actual_plate[ : , 5 : 42]
    numbers = actual_plate[ : , 58 : 95]

    letterOne = letters[:, :18]
    letterTwo = letters[:, 18 : 36]

    numberOne = numbers[:, :18]
    numberTwo = numbers[:, 18 : 36]
        
    cv2.imshow("alphaOne", letterOne)
    cv2.waitKey(2)
        
    cv2.imshow("alphaTwo", letterTwo)
    cv2.waitKey(2)
        
    cv2.imshow("numberOne", numberOne)
    cv2.waitKey(2)
        
    cv2.imshow("numberTwo", numberTwo)
    cv2.waitKey(2)

    userInput = int(input("Put in plate # or 0 if you would like to skip"))

    if userInput >= 1 and userInput <= 8:
        print(np.shape(letterOne))
        print(np.shape(letterTwo))
        print(np.shape(numberOne))
        print(np.shape(numberTwo))
        print(np.shape(parkingSpot))
        cv2.imwrite(os.path.dirname(os.path.realpath(__file__)) + '/plate/letters/' + str(letters[userInput][0]) + '-' +  str(self.imNum) + '.png', letterOne)
        self.imNum += 1
        cv2.imwrite(os.path.dirname(os.path.realpath(__file__)) + '/plate/letters/' + str(letters[userInput][1]) + '-' +  str(self.imNum) + '.png', letterTwo)
        self.imNum += 1
        cv2.imwrite(os.path.dirname(os.path.realpath(__file__)) + '/plate/numbers/' + str(numbers[userInput][0]) + '-' +  str(self.imNum) + '.png', numberOne)
        self.imNum += 1
        cv2.imwrite(os.path.dirname(os.path.realpath(__file__)) + '/plate/numbers/' + str(numbers[userInput][1]) + '-' +  str(self.imNum) + '.png', numberTwo)
        self.imNum += 1
        cv2.imwrite(os.path.dirname(os.path.realpath(__file__)) + '/plate/parking/' + str(userInput) + '-' +  str(self.imNum) + '.png', parkingSpot)
        self.imNum += 1

def cleanImage(image, kernel):
    image = cv2.erode(image, kernel, iterations=1)
    return cv2.dilate(image, kernel, iterations=1)

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
	M = cv2.getPerspectiveTransform(rect, dst)
	warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
	warped = cv2.resize(warped, (100, 200), interpolation = cv2.INTER_CUBIC)
	# return the warped image
	return warped

def main(args):
    lpd = license_plate_detector()
    rospy.init_node('license_plate_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)