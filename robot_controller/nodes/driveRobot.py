#!/usr/bin/env python
import rospy
import sys
import roslib
roslib.load_manifest('robot_controller')
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

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
    
    #self.image_pub = rospy.Publisher("/cmd_vel",Twist)
    self.image_pub = rospy.Publisher("/R1/cmd_vel",Twist)

    time.sleep(1)

    lh = 0
    ls = 0
    lv = 83
    uh = 255
    us = 255
    uv = 86
    self.lower_hsv_b = np.array([lh,ls,lv])
    self.upper_hsv_b = np.array([uh,us,uv])

  
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

    #We can either create a separate time node or include time somewhere in this
    #self.image_sub = rospy.Subscriber("/clock", time, self.callback)

    move = Twist()
    
    move.linear.x = 0.2
    move.angular.z = 1

    #THIS IS TO START DRIVING
    #self.image_pub.publish(move)

    time.sleep(3)






  def callback(self,data):

    move = Twist()
  
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)


    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    img = cv2.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) // 255

    #road is lighter than the backdrop, maybe try thresholding between 50-100
    #then taking the different between them
  
    threshold = 90
    #_, img = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    #this one
    #_, img = cv2.threshold(gray, threshold, 255, 0 )

    #img = img[700:800]
    img = img[0:int(img.shape[0]*0.7),int(img.shape[1]*0.1):int(img.shape[1]*0.9)]
    gray = gray[0:int(img.shape[0]*0.7),int(img.shape[1]*0.1):int(img.shape[1]*0.9)]
    

    img = cv2.erode(img, None, iterations = 2)

    #I dont think we wanna invert anymore? JK WE DEF DO
    #img = np.invert(img)

    cX = int(img.shape[1]*0.5)
    cY = 400

    M = cv2.moments(img)
    '''
    try:
      cX = int(M["m10"] / M["m00"])
      #cY = int(M["m01"] / M["m00"]) + 700
    except ZeroDivisionError:
      pass
    '''

    if(M["m00"] != 0):
            
      #print("1")
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      #print("2")
      #self.timeout = 0
      #print("cX:")
      #print("CX: {:}".format(cX))
            
      #print("3")
      #width = np.shape(gray)[1]
      #im_width = width // 10
      #print("width:")
      #print(width)

      #subsection = int(cX / im_width)
            
      #print("subsection: {:}".format(subsection))
      #print(cX)

      #state[subsection] = 1
      '''
    else:
      self.timeout += 1
      '''

    # color = cv2.cvtColor(hsv, cv.COLOR_BGR2HSV)
    '''
    out.write(color)
    '''
    cv2.circle(gray, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
    cv2.imshow("img", gray)
    cv2.waitKey(2)

    VelWeight = 15*8 #270
    cX = 1*(cX - img.shape[1]*0.5)/VelWeight

    '''
    if cX == 0:
      cX = 1.0
    '''

    print(cX)

    

    move.linear.x = 0.2
    move.angular.z = cX


    #THIS IS REQUIRED FOR DRIVING
    self.image_pub.publish(move)

    '''
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    '''
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  
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