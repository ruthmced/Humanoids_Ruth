#!/usr/bin/env python
import rospy
import numpy
import cv2
import ros_numpy
import math
import random
import argparse
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge   
from matplotlib import pyplot as plt

def callback_image (msg):
    print ("imagen recibida")
    bridge = CvBridge() 
    #Read de original image 
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#source file
   
    # Display original image
    cv2.imshow("imagen",cv_image) #source file
    cv2.waitKey(10)

    
    # Convert to graycsale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) 
    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(gray, (3,3), 0) 
    #--------------------------FIND EDGES*------------------------------------#
    # Sobel Edge Detection!!!!!!!!!!!!!!!
    sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
    sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
    sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
    
    
    # Canny Edge Detection
    edges = cv2.Canny(img_blur,100, 200) # Canny Edge Detection
    # Display Canny Edge Detection Image
    cv2.imshow('Canny Edge Detection', edges)#*
    cv2.waitKey(10)
    
    #-------------------------FIND CONTOURS**------------------------------#
    # apply binary thresholding
    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)#0 is black 255 is white

    # visualize the binary image
    cv2.imshow('Binary image', thresh)
    cv2.waitKey(10)
    cv2.imwrite('cv_image_thres1.jpg', thresh)
    cv2.imwrite('cv_image.jpg', cv_image)
    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.INTERSECT_FULL, method=cv2.CHAIN_APPROX_NONE)
                                        
    # draw contours on the original image
    image_copy = cv_image.copy()
    cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    cnt=contours[0]
    M = cv2.moments(cnt)
    print( M )              
    # see the results
    cv2.imshow('None approximation', image_copy)
    cv2.waitKey(10)
    cv2.imwrite('contours_none_image1.jpg', image_copy)


def main ():
    rospy.init_node("goal_detection_node")  
    rospy.Subscriber("/hardware/camera/image", Image, callback_image)
    rospy.spin()

if __name__=="__main__":
    main()
