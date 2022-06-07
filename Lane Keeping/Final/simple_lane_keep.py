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
    #Convert the image to GRAY
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Add a GaussianBlur to blur the edges for better edge detection
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    # Extract the lines and place into an image called canny
    canny = cv2.Canny(blur, threshold1=50, threshold2=200, apertureSize=3)
    # ---
    return canny

def region_of_interest(image):
    #create vertices for shape at the following points
    rectangle = np.array([[(0, 480),(0, 288), (639, 288), (639, 480)]])
    # TO:DO Find the  Region of Interest
    # ---
    #create an object like the image (shape) and fill with zeros
    mask = np.zeros_like(image)
    #fill a polygon with the above vertices with color white and place in object mask
    cv2.fillPoly(mask, rectangle, 255)
    #logical AND the image and the mask to created an ROI image
    masked_image = cv2.bitwise_and(image, mask)
    # ---
    return masked_image

def average_slope_intercept(image, lines):
    # TO-DO: Get and average of the left and right Hough Lines and extract the centerline. 
    # The angle between the extracted centerline and desired centerline will be the error. 
    # Use cv2.line to display the lines.
    # ---
    # create an empty array for left lines and right lines
    left = []
    right = []
    for line in lines:
        #extract the data points from the lines
        x1, y1, x2, y2 = line.reshape(4)
        # create a line in y=mx+b from the each set of points
        realizedLine = np.polyfit((x1, x2), (y1,y2), 1)
        # extract the slope of each line 
        m = realizedLine[0]
        #extract the y intercept from each line
        b = realizedLine[1]
        
        # Determines if line is a left line or a right line
        if m < 0:
            # Because OpenCV uses the top left as the origin
            # left lines have a negative slope and right lines
            # have a positive slope
            left.append((m, b))
        else:
            right.append((m, b))
            
    # Take the average of all left lines
    avgLeft = np.average(left, axis=0)
    # Take the average of all right lines
    avgRight = np.average(right, axis=0)
    # Extract the x1,x2,y1,y2 points from each left and right lines
    leftLine = get_line_points(image, avgLeft)  
    rightLine = get_line_points(image, avgRight)  
    return np.array([leftLine, rightLine])
    # ---
    
def get_line_points(image, line):
    #Take the image and the averaged line to calculate its x1,x2,y1,y2 points
    m, b = line
    #y1 is the height (480)
    y1 = image.shape[0]
    
    #y2 = 200 points down from y1. This is a set value and will not change
    #200 is chosen to the lines always in view.
    y2=int(y1 - 200)
    #calculating x1 and x2 from y=mx+b for where x = (y-b)/m
    x1=int((y1-b)/m)
    x2=int((y2-b)/m)
    
    #returns the points as a numpy array
    return np.array([x1, y1, x2, y2])


def draw_lines(image, lines):
    # Create an empty window with the same shape as original image
    linesDrawn = np.zeros_like(image)
    # As longer as there is a line to print, do loop
    if lines is not None:
        # For each set of points in array
        for x1, y1, x2, y2 in lines:
            # Print each line using the data points in RED
            cv2.line(linesDrawn, (x1,y1), (x2,y2), (0,0,255), 5)
    # Returns the image with the drawn lines 
    return linesDrawn

def get_centerline(lines):
   # Finds the centerline by averaging the left and right lines
   if len(lines)>0:
       # Returns the centerline
       return np.mean(lines, axis=0, dtype=np.int32)
   else:
       # If there are no lines, return nothing
       return None
        
def camera_callback(data):
    global steering_angle
    global velocity
    
    # TO-DO: Convert the ROS Image to CV type.
    # ---
    try:
        # Take the subscriber data and turn it into an image we can use in OpenCV
    	cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        # If we are unable to get the image, error out
    	print(e)
    	
    # TO-DO: Extract the canny lines
    canny_image = canny(cv_image)

    # TO:DO Find the  Region of Interest
    cropped_image = region_of_interest(canny_image)
    
    # Extract the Hough Lines
    # Does the Probabilistic Hough Transform and places lines into variable named hough
    # The arguments used produced the best results 
    hough = cv2.HoughLinesP(cropped_image, rho=1, theta=np.pi / 180, threshold=80, minLineLength=50, maxLineGap=10)
    # Take calculated hough lines and find the average left and right lines and their respective data points
    averaged_lines = average_slope_intercept(cv_image, hough)
    # Find the centerline
    centerline = get_centerline(averaged_lines)
    # Extract the coordinates for the centerline
    x1,y1,x2,y2 = centerline

    # Draw the average left and right lines onto the houghLines image 
    houghLines = draw_lines(cv_image, averaged_lines)
    # Draw the centerline on the same image as the averaged lines
    cv2.line(houghLines, (x1,y1), (x2,y2), (0,0,255),3)
    # Create an image of the original camera view with the lines added 
    houghLines = cv2.addWeighted(cv_image, 0.9, houghLines, 1,1)      

    # TO-DO: Implement the final controller
    # ---
    kp = 0.9
    # The desired angle we want from the centerline
    desiredAngle = math.radians(80)
    # The slope of the centerline
    m = (y2-y1)/(x2-x1)
    print(f"Centerline slope is: {m}")
    
    # Finding the actual slope of the centerline
    actualAngle = math.atan(m)
    
    #print(f"actual angle is: {actualAngle}")
    
    # Error is the desired centerline angle - the actual angle of the centerline
    err = desiredAngle - abs(actualAngle)
    
    # Steering angle is the error * kp (tuned parameter)
    steering_angle_c = err*kp
    print(f"Steering angle is: {steering_angle_c}")
    
    # Creates the AckermannDriveStamped msg 
    msg = AckermannDriveStamped()
    # Applay the velocity 
    msg.drive.speed = velocity
    # Apply the steering angle found above
    msg.drive.steering_angle = steering_angle_c
    # Publish the message
    pub.publish(msg)
    # ---
    
    # Display all images images
    cv2.imshow('original', cv_image)
    cv2.imshow('canny',canny_image)
    cv2.imshow('ROI',cropped_image)
    cv2.imshow('Hough Lines w/ Centerline', houghLines)
    cv2.waitKey(1)

if __name__ == '__main__':
    print("Lane Keeping")
    rospy.init_node('lane_keeping',anonymous=True)
    # Create bridge between ROS and OpenCV
    bridge_object = CvBridge()

    # TO-DO: Publish and subscribe to the correct topics. 
    # ---
    # Set rate to be 10Hz
    rate = rospy.Rate(10)
    # Create publisher to send velocity and steering angle
    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 1)
    while not rospy.is_shutdown():
        # Subscribe to the camera data
    	rospy.Subscriber("/camera/zed/rgb/image_rect_color",Image,camera_callback)
    	rospy.spin()
    # ---
    
