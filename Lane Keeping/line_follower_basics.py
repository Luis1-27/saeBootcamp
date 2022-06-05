#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vel_msg = Twist()
        
    
    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            # Cropping Image
            height,width,channels = cv_image.shape
            crop_img = cv_image[int(height/2)+100:int(height/2)+140][1:width]
            
            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            
            #Define a range of the collor yellow
            lower_yellow = np.array([20,100,100])
            upper_yellow = np.array([50,255,255])
            
            #Threshold the HSV image to get only yellow
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            #Calculate centroid of the blob (cx,cy)
            m = cv2.moments(mask,False)
            try: 
            	cx,cy = m['m10']/m['m00'],m['m01']/m['m00']

            except ZeroDivisionError:
            	cx,cy = height/2, width/2
            
            # Draw the centroid in the resultant image
            cv2.circle(mask, (int(cx),int(cy)), 10,(0,0,255),-1)
            # twist stuff?
            Kp = 0.01
            
            # Calculate the error between the width of the image and the location of the centroid
            err = -float(cx - width/2.5)
            self.vel_msg.linear.x = 0.5
            self.vel_msg.angular.z = Kp*err
            self.velocity_publisher.publish(self.vel_msg)
            print(f"The error is: {err} pixels")
            print(f"angular.z: {Kp*err}")
            
            	
            
            
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
