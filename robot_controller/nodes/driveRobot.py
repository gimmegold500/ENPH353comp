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

    self.basespeedhigher = 0.18
    self.basespeedlower = 0.12

    self.flag = True

    self.startingdrive = True # edit here to start the way you want to 
    self.pedoseen = 0

    self.time_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    time.sleep(1)
    self.time_pub.publish("Bestie,Bestie,0,XR58")
    self.redcounter = 0
    self.redcounterdriving = 0
    self.whitecounter = 0
    self.firstsawthecar = 240

    self.turnleft = 0
    self.lookforintersections = False
    self.carwatching = 0
    self.stopduetograycar = 0

    time.sleep(1)

    #for the road
    lh = 0
    ls = 0
    lv = 80
    uh = 255
    us = 255
    uv = 88
    self.lower_hsv_b = np.array([lh,ls,lv])
    self.upper_hsv_b = np.array([uh,us,uv])

    # blue
    lhblue = 116
    lsblue = 40
    lvblue = 41
    uhblue = 124
    usblue = 255
    uvblue = 255
    self.lower_hsv_blue = np.array([lhblue,lsblue,lvblue])
    self.upper_hsv_blue = np.array([uhblue,usblue,uvblue])

    # dark thresholds
    lhdarkb = 0
    lsdarkb = 0
    lvdarkb = 94
    uhdarkb = 0
    usdarkb = 0
    uvdarkb = 140
    self.lower_hsv_w_dark = np.array([lhdarkb,lsdarkb,lvdarkb])
    self.upper_hsv_w_dark = np.array([uhdarkb,usdarkb,uvdarkb])

    # light thresholds
    lhlightb = 90
    lslightb = 0
    lvlightb = 74
    uhlightb = 123
    uslightb = 47
    uvlightb = 96
    self.lower_hsv_plate_dark = np.array([lhlightb, lslightb, lvlightb])
    self.upper_hsv_plate_dark = np.array([uhlightb, uslightb, uvlightb])

    # car_image thresholds
    lhcar = 0
    lscar = 0
    lvcar = 99
    uhcar = 255
    uscar = 1
    uvcar = 255
    self.lower_hsv_car = np.array([lhcar, lscar, lvcar])
    self.upper_hsv_car = np.array([uhcar, uscar, uvcar])

    self.lower_hsv_w_bright = np.array([0, 0, 191])
    self.upper_hsv_w_bright = np.array([0, 0, 206])
    self.lower_hsv_plate_bright = np.array([103, 0, 112])
    self.upper_hsv_plate_bright = np.array([155, 35, 184])

    self.count = 0
    self.intialtime = rospy.get_rostime().secs
    

    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
    self.pastimage = np.array([0])

    #We can either create a separate time node or include time somewhere in this
    #self.image_sub = rospy.Subscriber("/clock", time, self.callback)






  def callback(self,data):
    global previous_image
    currenttime = rospy.get_rostime().secs
    #print("Looping")

    move = Twist()


    #This is for stopping the timer
    # CHANGE THIS TO 4 min (240) BEFORE COMPETITION!!!!
    if(currenttime-self.intialtime >= 100):
      if(self.flag):
        self.time_pub.publish("Bestie,Bestie,-1,XR58")
        self.flag = False
      #print("THIS SHOULD NOW STOP THE TIMER")
      while(True):
        move.linear.x = 0
        move.angular.z = 0
        self.vel_pub.publish(move)

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

  


    #Pedestrian stuff here
    
    
    RedThresholdLower = np.array([0,113,253])
    RedThresholdHigher = np.array([255, 255,255])

    WhiteThresholdLower = np.array([0,0,254])
    WhiteThresholdHigher = np.array([255, 243, 255])

    PedoThresholdLower = np.array([100,0,43])
    PedoThresholdHigher = np.array([117, 255, 76])

    #driving stuff here, indexed at 1 to have a past image work for pedestrian crossing
    if(self.count > 1):
 
      currenthsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      #past image still isn't working, plz fix
      pasthsv = cv2.cvtColor(previous_image, cv2.COLOR_BGR2HSV)

      red_raw = cv2.inRange(currenthsv, RedThresholdLower, RedThresholdHigher)
      white_raw = cv2.inRange(currenthsv, WhiteThresholdLower, WhiteThresholdHigher)
      pedo_raw = cv2.inRange(currenthsv,PedoThresholdLower,PedoThresholdHigher)
      past_for_pedo_raw = cv2.inRange(pasthsv,RedThresholdLower, RedThresholdHigher)

      diswhiteline_image = red_raw // 255
      white_image = white_raw // 255
      current_pedo_image = pedo_raw // 255
      past_pedo_image = past_for_pedo_raw // 255
      
  

      diswhiteline_image = diswhiteline_image[int(diswhiteline_image.shape[0]*0.9):diswhiteline_image.shape[0],int(
        diswhiteline_image.shape[1]*0.2):int(diswhiteline_image.shape[1]*0.8)]
      white_image = white_image[int(white_image.shape[0]*0.5):int(white_image.shape[0]*0.8),int(
        white_image.shape[1]*0.45):int(white_image.shape[1]*0.55)]
      current_pedo_image = current_pedo_image[int(current_pedo_image.shape[0]*0.5):int(
        current_pedo_image.shape[0]*0.7),int(current_pedo_image.shape[1]*0.28):int(current_pedo_image.shape[1]*0.6)]
      past_pedo_image = past_pedo_image[int(past_pedo_image.shape[0]*0.5):int(
        past_pedo_image.shape[0]*0.7),int(past_pedo_image.shape[1]*0.28):int(past_pedo_image.shape[1]*0.6)]
     
      red_raw = red_raw[int(red_raw.shape[0]*0.4):int(red_raw.shape[0]*0.6),int(
        red_raw.shape[1]*0.4):int(red_raw.shape[1]*0.6)]
      white_raw = white_raw[int(white_raw.shape[0]*0.5):int(white_raw.shape[0]*0.8),int(
        white_raw.shape[1]*0.45):int(white_raw.shape[1]*0.55)]
      pedo_raw = pedo_raw[int(pedo_raw.shape[0]*0.5):int(
        pedo_raw.shape[0]*0.7),int(pedo_raw.shape[1]*0.28):int(pedo_raw.shape[1]*0.6)]
      past_for_pedo_raw = past_for_pedo_raw[int(past_for_pedo_raw.shape[0]*0.5):int(
        past_for_pedo_raw.shape[0]*0.7),int(past_for_pedo_raw.shape[1]*0.28):int(past_for_pedo_raw.shape[1]*0.6)]

      difference_image = current_pedo_image - past_pedo_image
      difference_raw = pedo_raw - past_for_pedo_raw
      
      
      if((np.sum(white_image) > 1500)):
        print("White val:")
        print(np.sum(white_image))
        self.whitecounter = 15

      if(self.whitecounter > 0):
        self.whitecounter -= 1
      
      if(np.sum(difference_raw > 10000)):
        
        print("Pedo moving with:")
        print(np.sum(difference_raw))

      #cv2.imshow("difference", difference_raw)
      #cv2.waitKey(2)

      #cv2.imshow("White", white_raw)
      #cv2.waitKey(2)

      #cv2.imshow("Red", red_raw)
      #cv2.waitKey(2)

      if(np.sum(diswhiteline_image) > 10000):
        self.redcounter = 10
        self.redcounterdriving = 15

      if(self.redcounter > 0 and self.pedoseen == 0):
        self.redcounter -= 1
        print("Red stop")

      if(self.redcounterdriving > 0):
        self.redcounterdriving -= 1

      #driving code here as well as pedestrian stuff
      if((self.redcounter > 0 
            and (np.sum(white_image) > 2000) 
            #and (np.sum(current_pedo_image - past_pedo_image) > 150) 
            and (np.sum(difference_raw) > 50000 )) or self.pedoseen > 0 ) :

        if(self.pedoseen <= 0):
          self.pedoseen = 10
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
      elif(self.stopduetograycar > 0 ):
        carphoto = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        car_raw= cv2.inRange(carphoto, self.lower_hsv_car, self.upper_hsv_car)
        car_image = car_raw // 255

        car_image =  car_image[0:car_image.shape[0],int(car_image.shape[1]*0):int(car_image.shape[1]*0.6)]
        car_raw =  car_raw[0:car_raw.shape[0],int(car_raw.shape[1]*0):int(car_raw.shape[1]*0.6)]

        cv2.imshow("looking for grey car", car_raw)
        cv2.waitKey(2)

        print("seeing if car_image closeby")
        print(np.sum(car_image))
        print(self.stopduetograycar)

        #move.linear.x = 0
        #move.angular.z = 0
        #self.vel_pub.publish(move)
        print("STOPPING DUE TO GRAY CAR WATCHING")

        if(np.sum(car_image) > 50000):
          #cv2.imshow("car", car_raw)
          #cv2.waitKey(2)
          #print(np.sum(car_image))
          print("STOPPING DUE TO GRAY CAR")
          self.stopduetograycar = 10
        
        #self.carwatching = 10

        if(self.stopduetograycar == 1):
          move.linear.x = 0.2
          move.angular.z = 0.9

          #THIS IS TO START DRIVING
          self.vel_pub.publish(move)
          now = rospy.get_rostime().secs
          while(rospy.get_rostime().secs - now < 1):
            print("Left turn")

        if(self.stopduetograycar > 0):
          self.stopduetograycar -= 1


      elif(self.redcounterdriving > 0 and (self.whitecounter) > 0):
        self.turnleft += 1

        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_raw= cv2.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) 
        img = img_raw //255

        img = img[int(img.shape[0]*0.6):img.shape[0],int(img.shape[1]*0.2):int(img.shape[1]*0.8)]
        gray = gray[int(gray.shape[0]*0.6):gray.shape[0],int(gray.shape[1]*0.2):int(gray.shape[1]*0.8)]
        img_raw = img_raw[int(img_raw.shape[0]*0.6):img_raw.shape[0],int(img_raw.shape[1]*0.2):int(img_raw.shape[1]*0.8)]

        img = cv2.erode(img, None, iterations = 4)
        img_raw = cv2.erode(img_raw, None, iterations = 4)

        #Just incase moments is 0 
        cX = int(img.shape[1]*0.5*0.5)
        cY = int(img.shape[1]*0.8*0.5)


        M = cv2.moments(img)
        
        #calculating moments
        if(M["m00"] != 0):
          cX = int(M["m10"] / M["m00"])
          cY = int(M["m01"] / M["m00"])

      
        #Next lines until break are all for testing
        cv2.circle(gray, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
        cv2.imshow("img", gray)
        cv2.waitKey(2)

        #How good the PID is
        VelWeight = 215 #110
        cX = 1*(cX - img.shape[1]*0.5)/VelWeight

        #for Testing
        print(cX)

        move.linear.x = 0.45
        move.angular.z = -1*cX

        print("fast driving")

        self.vel_pub.publish(move)
      else:
        
        #if(self.carwatching == 10):
          #self.carwatching -= 1
        

        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_raw= cv2.inRange(hsv, self.lower_hsv_b, self.upper_hsv_b) 
        img = img_raw // 255

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

        #cv2.circle(img_raw, (cX,cY), radius=0, color=(0, 0, 255), thickness = 50)
        #cv2.imshow("img_raw", img_raw)
        #cv2.waitKey(2)


        #How good the PID is
        VelWeight = 215 #110
        cX = 1*(cX - img.shape[1]*0.5)/VelWeight

        #for Testing
        print(cX)



        
        if(abs(cX) < 0.4):
          move.linear.x = self.basespeedhigher 
          move.angular.z = -1*cX
        else:
          move.linear.x = self.basespeedlower 
          move.angular.z = -1*cX


        #THIS IS REQUIRED FOR DRIVING
        self.vel_pub.publish(move)
      
  

    #After last red line, starts looking for last Car
    if(self.turnleft > 25):
      print("turnleft:")
      print(self.turnleft)

      imgforbluecar = self.bridge.imgmsg_to_cv2(data, "bgr8")
      height = imgforbluecar.shape[0]
      width = imgforbluecar.shape[1]
      imgforbluecar = imgforbluecar[height // 2 :  (3 * height) // 4]
      hsv = cv2.cvtColor(imgforbluecar, cv2.COLOR_BGR2HSV)

      # get masks
      mask_blue = cv2.inRange(hsv, self.lower_hsv_blue, self.upper_hsv_blue) // 255
      mask_white_dark = cv2.inRange(hsv, self.lower_hsv_w_dark, self.upper_hsv_w_dark) // 255
      mask_white_bright = cv2.inRange(hsv, self.lower_hsv_w_bright, self.upper_hsv_w_bright) // 255
      mask_plate_dark = cv2.inRange(hsv, self.lower_hsv_plate_dark, self.upper_hsv_plate_dark) // 255
      mask_plate_bright = cv2.inRange(hsv, self.lower_hsv_plate_bright, self.upper_hsv_plate_bright) // 255

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

      # process car_image if needed
        
      if car_is_spotted(self, mask_blue_l, mask_white_dark_l, mask_plate_dark_l):
          #process_car(self, mask_blue_l, mask_white_dark_l, mask_plate_dark_l, img[:, 0:width//2], kernel_3)

          print("light TRUE")
          self.firstsawthecar = rospy.get_rostime().secs
          self.turnleft = 0
          self.lookforintersections = True

    #After seeing last car starts looking for Intersections
    if(self.lookforintersections and (rospy.get_rostime().secs - self.firstsawthecar) > 3 ):

      WhiteThresholdLower = np.array([0,0,254])
      WhiteThresholdHigher = np.array([255, 243, 255])

      currenthsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

      diswhiteline_raw = cv2.inRange(currenthsv, WhiteThresholdLower, WhiteThresholdHigher)

      diswhiteline_image = diswhiteline_raw // 255

      diswhiteline_image = diswhiteline_image[int(diswhiteline_image.shape[0]*0.2):diswhiteline_image.shape[0],int(
        diswhiteline_image.shape[1]*0.2):int(diswhiteline_image.shape[1]*0.4)]


      diswhiteline_raw = diswhiteline_raw[int(diswhiteline_raw.shape[0]*0.2):diswhiteline_raw.shape[0],int(
        diswhiteline_raw.shape[1]*0.2):int(diswhiteline_raw.shape[1]*0.4)]


      #cv2.imshow("looking for white line", diswhiteline_raw)
      #cv2.waitKey(2)

      print(np.sum(diswhiteline_image))

      if(np.sum(diswhiteline_image) < 1000):
        self.basespeedhigher = 0.15
        self.basespeedlower = 0.10
          
        #self.carwatching -= 1

        move.linear.x = 0.2
        move.angular.z = 0.9

        #THIS IS TO START DRIVING
        self.vel_pub.publish(move)
        now = rospy.get_rostime().secs
        while(rospy.get_rostime().secs - now < 2):
          print("Left turn")

        move.linear.x = 0
        move.angular.z = 0
        self.vel_pub.publish(move)

        self.stopduetograycar = 10

        self.lookforintersections = False



    self.count += 1
    

    #Remove this when done
    '''
    if(self.count == 10):
      print("SWITCH STATEMENT")
      self.carwatching = 10
    '''

    previous_image = willBePast

    '''
    try:
      self.vel_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    '''


  
def cleanImage(image, kernel):
    image = cv2.erode(image, kernel, iterations=1)
    return cv2.dilate(image, kernel, iterations=1)


def car_is_spotted(self, blue_vals, white_vals, grey_vals):
    print(np.sum(grey_vals))
    return np.sum(blue_vals) > 25000 and np.sum(blue_vals) < 37500 and np.sum(white_vals) > 500

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


'''
elif(self.carwatching > 0 
#and self.carwatching != 10
):
  
  carphoto = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  car_raw= cv2.inRange(carphoto, self.lower_hsv_car, self.upper_hsv_car)
  car_image = car_raw // 255

  car_image =  car_image[0:car_image.shape[0],int(car_image.shape[1]*0.4):int(car_image.shape[1])]
  car_raw =  car_raw[0:car_raw.shape[0],int(car_raw.shape[1]*0.4):int(car_raw.shape[1])]

  cv2.imshow("car", car_raw)
  cv2.waitKey(2)

  print("seeing if car_image closeby")
  print(np.sum(car_image))

  
  move.linear.x = 0
  move.angular.z = 0
  self.vel_pub.publish(move)
  

  if(np.sum(car_image) > 50000):

    while(np.sum(car_image) > 50000):
      cv2.imshow("car", car_raw)
      cv2.waitKey(2)
      print(np.sum(car_image))
      print("STOPPING DUE TO GRAY CAR")
      self.carwatching = 10
      
      #move.linear.x = 0
      #move.angular.z = 0
      #self.vel_pub.publish(move)
      
    
  self.carwatching -= 1

'''

      