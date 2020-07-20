#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import time

bridge = CvBridge()

def image_conv(image1):
  try:
    cv_image = bridge.imgmsg_to_cv2(image1, "bgr8")
  except CvBridgeError as e:
    print(e)
  return cv_image

def colour_filter(image2):

  hsv = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)
  yellowlower = (30,120,100)
  yellowupper = (60,225,225)
  mask = cv2.inRange(hsv, yellowlower, yellowupper)
  return mask

def contour_detect(image3):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
  contours = cv2.findContours(image3.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
  return contours



def draw_ball_contour(binary_image, rgb_image, contours):
  black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
  for c in contours[0]:
      area = cv2.contourArea(c)
      perimeter= cv2.arcLength(c, True)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      if (area>3000):
          cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
          cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
          cx, cy = get_contour_center(c)
          cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
          cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
          cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
          #print ("Area: {}, Perimeter: {}".format(area, perimeter))
  #print ("number of contours: {}".format(len(contours)))
  cv2.imshow("RGB Image Contours",rgb_image)
  cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def image_callback(ros_image):
  print 'Image received'
  global bridge
  
  #convert ros_image into an opencv-compatible image
  conv_img = image_conv(ros_image)     

  #apply colour filtering to recognise the yellow ball,returns a binary image 
  masked_img = colour_filter(conv_img) 
  
  #apply contour detection on the masked image
  detected_contours = contour_detect(masked_img)

  draw_ball_contour(masked_img, conv_img, detected_contours)
  cv2.imshow("Image window", masked_img)
  cv2.waitKey(3)

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
