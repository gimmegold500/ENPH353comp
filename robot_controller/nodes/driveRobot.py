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


previous_image = None

class image_converter:



  def __init__(self):
    
    #self.vel_pub = rospy.Publisher("/cmd_vel",Twist)
    self.vel_pub = rospy.Publisher("/R1/cmd_vel",Twist)


    self.flag = True

    self.startingdrive = True #False
    self.pedoseen = 0

    self.time_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    time.sleep(1)
    self.time_pub.publish("TeamA,aileton,0,XR58")
    self.redcounter = 0

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
    self.pastimage = np.array([0])

    #We can either create a separate time node or include time somewhere in this
    #self.image_sub = rospy.Subscriber("/clock", time, self.callback)





  '''
  def Stopifred(current,past,self):
    RedThresholdLower = np.array([0,113,253])
    RedThresholdHigher = np.array([255, 255,255])
    currenthsv = cv2.cvtColor(current, cv2.COLOR_BGR2HSV)
    #pasthsv = cv2.cvtColor(past, cv2.COLOR_BGR2HSV)
    red_raw = cv2.inRange(currenthsv, RedThresholdLower, RedThresholdHigher)
    #past_for_pedo_raw = cv2.inRange(pasthsv,RedThresholdLower, RedThresholdHigher)
    red_image = red_raw // 255
    #past_img = past_for_pedo_raw // 255

    red_image = red_image[int(red_image.shape[0]*0.9):red_image.shape[0],int(red_image.shape[1]*0.2):int(red_image.shape[1]*0.8)]
    #past_img = past_img[int(past_img.shape[0]*0.9):past_img.shape[0],int(past_img.shape[1]*0.2):int(past_img.shape[1]*0.8)]
    red_raw = red_raw[int(red_raw.shape[0]*0.9):red_raw.shape[0],int(red_raw.shape[1]*0.2):int(red_raw.shape[1]*0.8)]
    #past_for_pedo_raw = past_for_pedo_raw[int(past_for_pedo_raw.shape[0]*0.9):past_for_pedo_raw.shape[0],int(past_for_pedo_raw.shape[1]*0.2):int(past_for_pedo_raw.shape[1]*0.8)]

    if(np.sum(red_image) > 1000):
      move.linear.x = 0
      move.angular.z = 0
      self.vel_pub.publish(move)
  '''


  def callback(self,data):
    global previous_image
    currenttime = rospy.get_rostime().secs
    #print("Looping")

    move = Twist()

    if(self.startingdrive):
      move.linear.x = 0.22
      move.angular.z = 1.3

      #THIS IS TO START DRIVING
      self.vel_pub.publish(move)
      now = rospy.get_rostime().secs
      while(rospy.get_rostime().secs - now < 1):
        print("Starting turn")
      
      self.startingdrive = False
  
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    willBePast = cv_image
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)


  
    #This is for stopping the timer
    # CHANGE THIS TO 4 min (240) BEFORE COMPETITION!!!!
    if(currenttime-self.intialtime >= 240):
      if(self.flag):
        self.time_pub.publish("TeamA,aileton,-1,XR58")
        self.flag = False
      #print("THIS SHOULD NOW STOP THE TIMER")
      move.linear.x = 0
      move.angular.z = 0
      self.vel_pub.publish(move)


    #Pedestrian stuff here
    
    #Stopifred(cv_image,self.pastimage,self)
    RedThresholdLower = np.array([0,113,253])
    RedThresholdHigher = np.array([255, 255,255])

    WhiteThresholdLower = np.array([0,0,254])
    WhiteThresholdHigher = np.array([255, 243, 255])

    PedoThresholdLower = np.array([100,0,43])
    PedoThresholdHigher = np.array([117, 255, 76])


    if(self.count > 1):
 
      currenthsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      #past image still isn't working, plz fix
      pasthsv = cv2.cvtColor(previous_image, cv2.COLOR_BGR2HSV)

      red_raw = cv2.inRange(currenthsv, RedThresholdLower, RedThresholdHigher)
      white_raw = cv2.inRange(currenthsv, WhiteThresholdLower, WhiteThresholdHigher)
      pedo_raw = cv2.inRange(currenthsv,PedoThresholdLower,PedoThresholdHigher)
      past_for_pedo_raw = cv2.inRange(pasthsv,RedThresholdLower, RedThresholdHigher)

      red_image = red_raw // 255
      white_image = white_raw // 255
      current_pedo_image = pedo_raw // 255
      past_pedo_image = past_for_pedo_raw // 255
      
  

      red_image = red_image[int(red_image.shape[0]*0.9):red_image.shape[0],int(
        red_image.shape[1]*0.2):int(red_image.shape[1]*0.8)]
      white_image = white_image[int(white_image.shape[0]*0.5):int(white_image.shape[0]*0.8),int(
        white_image.shape[1]*0.45):int(white_image.shape[1]*0.55)]
      current_pedo_image = current_pedo_image[int(current_pedo_image.shape[0]*0.5):int(
        current_pedo_image.shape[0]*0.6),int(current_pedo_image.shape[1]*0.45):int(current_pedo_image.shape[1]*0.55)]
      past_pedo_image = past_pedo_image[int(past_pedo_image.shape[0]*0.5):int(
        past_pedo_image.shape[0]*0.6),int(past_pedo_image.shape[1]*0.45):int(past_pedo_image.shape[1]*0.55)]
     
      red_raw = red_raw[int(red_raw.shape[0]*0.4):int(red_raw.shape[0]*0.6),int(
        red_raw.shape[1]*0.4):int(red_raw.shape[1]*0.6)]
      white_raw = white_raw[int(white_raw.shape[0]*0.5):int(white_raw.shape[0]*0.8),int(
        white_raw.shape[1]*0.45):int(white_raw.shape[1]*0.55)]
      pedo_raw = pedo_raw[int(pedo_raw.shape[0]*0.5):int(
        pedo_raw.shape[0]*0.6),int(pedo_raw.shape[1]*0.45):int(pedo_raw.shape[1]*0.55)]
      past_for_pedo_raw = past_for_pedo_raw[int(past_for_pedo_raw.shape[0]*0.5):int(
        past_for_pedo_raw.shape[0]*0.6),int(past_for_pedo_raw.shape[1]*0.45):int(past_for_pedo_raw.shape[1]*0.55)]

      difference_image = current_pedo_image - past_pedo_image
      difference_raw = pedo_raw - past_for_pedo_raw
      
      

      
      
      #if((np.sum(white_image) > 1000)):
        #print("White stop")
      
      
      if(np.sum(difference_raw > 100)):
        print("Pedo moving")

      cv2.imshow("difference", difference_raw)
      cv2.waitKey(2)

      cv2.imshow("Red", red_raw)
      cv2.waitKey(2)

      if(np.sum(red_image) > 10000):
        self.redcounter = 6

      if(self.redcounter > 0):
        self.redcounter -= 1
        print("Red stop")

      
      
    
      if((self.redcounter > 0 
            #and (np.sum(white_image) > 1000) 
            #and (np.sum(current_pedo_image - past_pedo_image) > 150) 
            and (np.sum(difference_raw) > 100 )) or self.pedoseen > 0 ) :

        if(self.pedoseen <= 0):
          self.pedoseen = 20
        else:
          self.pedoseen -= 1
        
        print("difference: ")
        print(np.sum(difference_raw))
        print("pedo seen: ")
        print(self.pedoseen)
        move.linear.x = 0
        move.angular.z = 0
        self.vel_pub.publish(move)
        print("Should Be Stopped!!!")
      #'''
      else:

        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_raw= cv2.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) 
        img = img_raw //255

        #road is lighter than the backdrop, maybe try thresholding between 50-100
        #then taking the different between them
        #We dont use this anymore due to using HSV value filtering
        threshold = 90
        #_, img = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

        #this one
        #_, img = cv2.threshold(gray, threshold, 255, 0 )

        #img = img[700:800]
        img = img[int(img.shape[0]*0.6):img.shape[0],int(img.shape[1]*0.2):int(img.shape[1]*0.8)]
        gray = gray[int(gray.shape[0]*0.6):gray.shape[0],int(gray.shape[1]*0.2):int(gray.shape[1]*0.8)]
        img_raw = img_raw[int(img_raw.shape[0]*0.6):img_raw.shape[0],int(img_raw.shape[1]*0.2):int(img_raw.shape[1]*0.8)]

        img = cv2.erode(img, None, iterations = 4)
        img_raw = cv2.erode(img_raw, None, iterations = 4)

        #Don't invert anymore due to HSV filtering
        #img = np.invert(img)


        #Just incase moments is 0 
        cX = int(img.shape[1]*0.5*0.5)
        cY = int(img.shape[1]*0.8*0.5)


        M = cv2.moments(img)
        
        #calculating moments
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
        
        #else:
          #self.timeout += 1
        
          
        
      
        #Next lines until break are all for testing
        cv2.circle(gray, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
        cv2.imshow("img", gray)
        cv2.waitKey(2)

        cv2.circle(img_raw, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
        cv2.imshow("img_raw", img_raw)
        cv2.waitKey(2)


        #How good the PID is
        VelWeight = 215 #110
        cX = 1*(cX - img.shape[1]*0.5)/VelWeight

        #for Testing
        print(cX)

        move.linear.x = 0.2
        move.angular.z = -1*cX

        #THIS IS REQUIRED FOR DRIVING
        self.vel_pub.publish(move)
    #''' 
      
  

    self.count += 1

    previous_image = willBePast

    '''
    try:
      self.vel_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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


