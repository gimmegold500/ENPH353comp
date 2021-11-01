#!/usr/bin/env python
import rospy
import sys
import roslib
roslib.load_manifest('enph353_ros_lab')
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# rospy.init_node('view_robot', anonymous=True)

'''
from google.colab import drive
drive.mount('/content/drive')
'''
'''
shape = frame.shape
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('/content/drive/My Drive/ENPH 353/robotdriver.mp4', fourcc, 20, (shape[1], shape[0]))
'''

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/cmd_vel",Twist)



    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)


  def callback(self,data):

    move = Twist()
    

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)


    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

    threshold = 64
    _, img = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    img = img[700:800]

    img = cv2.erode(img, None, iterations = 2)
    img = np.invert(img)

    cX = 400
    cY = 400

    M = cv2.moments(img)
    try:
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"]) + 700
    except ZeroDivisionError:
      pass


    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    '''
    out.write(color)
    '''
    cX = ((cX - 400)*-1)/45

    if(cX == 0):
      cX= 1.0

    move.linear.x = 1
    move.angular.z = cX

    self.image_pub.publish(move)

'''
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
'''
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
'''
video.release()
out.release()
'''