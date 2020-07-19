#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

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


def contour_detect()

def image_callback(ros_image):
  print 'Image received'
  global bridge

  conv_img = image_conv(ros_image)     #convert ros_image into an opencv-compatible image
  masked_img = colour_filter(conv_img) #apply colour filtering to recognise the yellow ball 
                                       #returns a binary image   

  
  #apply contour detection on the masked image
  

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
