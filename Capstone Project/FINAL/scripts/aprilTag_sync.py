#!/usr/bin/env python3

import sys
import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

rospy.init_node('aprilTag_sync', anonymous=True)

    
pubImg = rospy.Publisher('/camera_rect/image_rect', Image, queue_size = 1)
pubInfo = rospy.Publisher('/camera_rect/camera_info', CameraInfo, queue_size=10)


def sync_callback(imgData, infoData):
  	
  pubImg.publish(imgData)
  pubInfo.publish(infoData)

if __name__ == '__main__':
  try:
  
    while not rospy.is_shutdown():
       image_sub_sync = message_filters.Subscriber('/raspicam_node/image', Image)
       info_sub_sync = message_filters.Subscriber('/raspicam_node/camera_info',CameraInfo)
       ts = message_filters.TimeSynchronizer([image_sub_sync, info_sub_sync], 10)
       ts.registerCallback(sync_callback)
       
       rospy.spin()
       
  except rospy.ROSInterruptException:
     pass
    
    
    
