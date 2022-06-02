#!/usr/bin/env python3

import rospy
import roslib
import sys
import cv2
import numpy as np
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import math

steering_angle = 0
velocity = 0.3 # (m/s)

def canny(image):
    # TO-DO: Extract the canny lines
    # ---
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    canny = cv2.Canny(blur, 50, 200, None, 3)
    # ---
    return canny

def region_of_interest(image):
    triangle = np.array([[(0, 480), (0, 288), (639, 288), (639, 480)]])
    # TO:DO Find the  Region of Interest
    # ---
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(image, mask)
    # ---
    return masked_image

#def average_slope_intercept(image, lines):
    # TO-DO: Get and average of the left and right Hough Lines and extract the centerline. 
    # The angle between the extracted centerline and desired centerline will be the error. 
    # Use cv2.line to display the lines.
    # ---

    
    
    # ---



def camera_callback(data):
    
    global steering_angle
    global velocity
    
    # TO-DO: Convert the ROS Image to CV type.
    # ---
    try:
    	cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
    	print(e)
    	
    # TO-DO: Extract the canny lines
    canny_image = canny(cv_image)

    # TO:DO Find the  Region of Interest
    cropped_image = region_of_interest(canny_image)
    
    # Adding a test image to see the hough lines
    houghP = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR)

    # Extract the Hough Lines
    lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 180, 150, None, 0, 0)


    # TO-DO: Get and average of the left and right Hough Lines and extract the centerline. 
    # The angle between the extracted centerline and desired centerline will be the error. 
    # Use cv2.line to display the lines.
    
    #if lines is None:
        #msg = AckermannDriveStamped()
        #msg.drive.speed = 1
        #pub.publish(msg)
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv2.line(houghP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
            
    #averaged_lines = average_slope_intercept(cv_image, lines)


    # TO-DO: Implement the final controller
    # ---

    # ---
    
    #rate = rospy.Rate(10)

    #while not rospy.is_shutdown():
        # TO-DO: Publish the steering angle and velocity
        # ---

        # ---

        #vel.header.stamp = rospy.Time.now()
        #vel.header.frame_id = "base_link"

        #print("Steering angle: %f" % m)
        #pub.publish(vel)

        #rate.sleep()

    #pub.publish(vel)
    #print("we got here111")
    # Display converted images
    cv2.imshow('original', cv_image)
    cv2.imshow('canny',canny_image)
    cv2.imshow('ROI',cropped_image)
    cv2.imshow('Hough', houghP)
    cv2.waitKey(1)
    #print("we got here")


if __name__ == '__main__':
    print("Lane Keeping")
    rospy.init_node('lane_keeping',anonymous=True)
    bridge_object = CvBridge()

    # TO-DO: Publish and subscribe to the correct topics. 
    # ---
    #rate = rospy.Rate(10)
    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 1)
    while not rospy.is_shutdown():
    	rospy.Subscriber("/camera/zed/rgb/image_rect_color",Image,camera_callback)
    	rospy.spin()
    # ---
    #vel = AckermannDriveStamped()
    
