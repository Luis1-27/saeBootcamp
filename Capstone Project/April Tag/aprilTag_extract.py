#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np

class aprilTagExtract(object):
  def __init__(self):
    #Creating our node,publisher and subscriber
    rospy.init_node('aprilTag_extract', anonymous=True)
    self.tag_publisher = rospy.Publisher('/chatter', String, queue_size=1)
    self.tag_subscriber = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.aprilTag_callback)
    self.rate = rospy.Rate(1)


  def aprilTag_callback(self,data):
      
      tagID = str(data.detections)
      aprilTagID = tagID[1:8]
      print(aprilTagID)
      self.tag_publisher.publish(aprilTagID)

def main():
  rospy.init_node('aprilTag_extract', anonymous=True)
  april_tag_object = aprilTagExtract()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
    
    
    
