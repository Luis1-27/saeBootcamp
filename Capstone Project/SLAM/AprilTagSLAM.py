#!/usr/bin/env python3

import rospy
import math
import move_base_seq
from std_msgs.msg import String

tag2cnt = 0

def apriltag_callback(data):
   
   global tag2cnt
   
   tagData = data.data
   
   if tagData == "id: [2]":
      tag2cnt = tag2cnt + 1
      
   if tag2cnt >= 1:
      move_base_seq.MoveBaseSeq()
   
   else:
      pass

if __name__ == '__main__':

    try:
       rospy.init_node('move_base_sequence')
       #rospy.init_node('apriltagslam',anonymous = True)
       while not rospy.is_shutdown():
          rospy.Subscriber("/chatter", String, apriltag_callback)
          
          rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("AprilTagSLAM finished.")
