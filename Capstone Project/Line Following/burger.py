#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.camera_callback)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vel_msg = Twist()
        
    
    def camera_callback(self,data):
        try:
        

            np_arr = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            height,width,channels = image_np.shape
            
            # crop image
            print(f"height: {height}")
            crop_img = image_np[int(height-40):int(height)][1:width]
            
            #convert to hsv
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # We select bgr8 because its the OpneCV encoding by default
            #cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            # Cropping Image
            #height,width,channels = cv_image.shape
            #crop_img = cv_image[int(height/2)+100:int(height/2)+140][1:width]
            
            # Convert from RGB to HSV
            #hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            
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
            Kp = 0.001
            print(f"cx: {cx}")
            
            # Calculate the error between the width of the image and the location of the centroid
            err = -float(cx - width/2)
            self.vel_msg.linear.x = 0.05
            self.vel_msg.angular.z = np.clip(Kp*err, -0.51, 0.51)
            self.velocity_publisher.publish(self.vel_msg)
            print(f"The error is: {err} pixels")
            print(f"angular.z: {self.vel_msg.angular.z}")
            
            	
            
            
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Original", image_np)
        cv2.imshow("Cropped", hsv)
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
