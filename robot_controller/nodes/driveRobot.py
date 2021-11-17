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

    self.time_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    time.sleep(1)
    self.time_pub.publish("TeamA,aileton,0,XR58")

    time.sleep(1)

    lh = 0
    ls = 0
    lv = 80
    uh = 255
    us = 255
    uv = 88
    self.lower_hsv_b = np.array([lh,ls,lv])
    self.upper_hsv_b = np.array([uh,us,uv])

    self.count = 0
    self.intialtime = rospy.get_rostime().secs

    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

    #We can either create a separate time node or include time somewhere in this
    #self.image_sub = rospy.Subscriber("/clock", time, self.callback)

    move = Twist()
    
    move.linear.x = 0.4
    move.angular.z = 1.3

    #THIS IS TO START DRIVING
    self.image_pub.publish(move)

    time.sleep(3)


  def callback(self,data):
    currenttime = rospy.get_rostime().secs
    
    move = Twist()
  
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)


    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    img_raw= cv2.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) 
    img = img_raw //255

    #road is lighter than the backdrop, maybe try thresholding between 50-100
    #then taking the different between them
  
    threshold = 90
    #_, img = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    #this one
    #_, img = cv2.threshold(gray, threshold, 255, 0 )

    #img = img[700:800]
    img = img[int(img.shape[0]*0.5):img.shape[0],int(img.shape[1]*0.2):int(img.shape[1]*0.8)]
    gray = gray[int(gray.shape[0]*0.5):gray.shape[0],int(gray.shape[1]*0.2):int(gray.shape[1]*0.8)]
    img_raw = img_raw[int(img_raw.shape[0]*0.5):img_raw.shape[0],int(img_raw.shape[1]*0.2):int(img_raw.shape[1]*0.8)]

    img = cv2.erode(img, None, iterations = 4)
    img_raw = cv2.erode(img_raw, None, iterations = 4)

    #Don't invert anymore due to HSV filtering
    #img = np.invert(img)

    cX = int(img.shape[1]*0.5*0.5)
    cY = int(img.shape[1]*0.4*0.5)

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

     

    #Next lines until break are all for testing
    cv2.circle(gray, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
    cv2.imshow("img", gray)
    cv2.waitKey(2)

    cv2.circle(img_raw, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
    cv2.imshow("img_raw", img_raw)
    cv2.waitKey(2)



    VelWeight = 110 #110
    cX = 1*(cX - img.shape[1]*0.5)/VelWeight

    #for Testing
    print(cX)

    

    move.linear.x = 0.2
    move.angular.z = -1*cX

    #This is for stopping the timer
    if(currenttime-intialtime == 120):
      self.time_pub.publish("TeamA,aileton,-1,XR58")
      print("THIS SHOULD NOW STOP THE TIMER")


    self.count += 1


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