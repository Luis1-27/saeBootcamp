#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
import time
import subprocess
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from object_recognition.msg import Predictor
from std_msgs.msg import String


global label
global counter
counter = 0
label = ''
tagData = ""
tag0cnt = 0
resnetFlag = False

class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.camera_callback)
        self.apriltag_sub = rospy.Subscriber('/chatter', String, self.apriltag_callback)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vel_msg = Twist()
        
    def apriltag_callback(self, data):
    
        global tagData
        
        tagData = data
        
    
    def camera_callback(self,data):
    
      global tagData
      global tag0cnt
      global resnetFlag
      
      #print(tagData.data)
      
      if tagData.data == "id: [0]":
         tag0cnt = tag0cnt + 1
         
      # if tag 1 or 2 are seen at least once then reset tag 0 counter and stop line following
      elif (tagData.data == "id: [1]") or (tagData.data == "id: [2]"):
         tag0cnt = 0
      if tag0cnt >= 1:
        
        if resnetFlag == False:
            print("got Here!")
            subprocess.Popen("rosrun aue_finals turtlebot_resnet.py", shell = True)
            self.pred_sub = rospy.Subscriber("/object_recognition", Predictor, self.pred_callback)
            resnetFlag = True
      
        try:
            
            cv_image = self.bridge_object.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
            height,width,channels = cv_image.shape
            
            # crop image
            print(f"height: {height}")
            crop_img = cv_image[int(height-40):int(height)][1:width]
            
            #convert to hsv
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
            Kp = 0.001
            
            print(f"label: {label}, counter: {counter}")
            if (label == 'street_sign') and (counter == 30):
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                print("we are at the stop sign waiting!!!!!!!!!!!")
                time.sleep(3)
            else:
                # Calculate the error between the width of the image and the location of the centroid
                err = -float(cx - width/2)
                self.vel_msg.linear.x = 0.05
                self.vel_msg.angular.z = np.clip(Kp*err, -0.51, 0.51)
                self.velocity_publisher.publish(self.vel_msg)
                print(f"The error is: {err} pixels")
                print(f"angular.z: {self.vel_msg.angular.z}")
                               
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Original", cv_image)
        cv2.imshow("Cropped", hsv)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)
      
      else: #if not then close windows, and reset the counter
        cv2.destroyAllWindows()
        tag0cnt = 0
        pass

    def pred_callback(self, data):
        global label 
        global counter
        
        label = data.label
        counter = data.score

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
