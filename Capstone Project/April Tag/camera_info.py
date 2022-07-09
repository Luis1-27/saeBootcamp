#!/usr/bin/env python3

import rospy
import cv2
from sensor_msg.msg import CameraInfo

rospy.init_node('camera_info', anonymous=True)

#def callback(dat):
#    q=CameraInfo()
#    print(q)
pub=rospy.Publisher('/camera_rect/camera_info', CameraInfo, queue_size=10)
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    q=CameraInfo()
#q.header.seq=3-
#q.headerstamp.sec=0
#q.header.stamp.nsecs=0
    q.header.frame_id="raspicam_node'
    q.height=304
    q.width=416
    #q.distortion_model='plumb_bob'
    q.D=[0.16635704844453678, -0.2734294401622768, -0.0008566139375164711, -0.003303571895196273, 0.0]
    q.K=[324.15649650890765, 0.0, 206.41974365780868, 0.0, 324.83848732368733, 154.62356617019483, 0.0, 0.0, 1.0]
    q.R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    q.P=[330.7861328125, 0.0, 204.55170960631222, 0.0, 0.0, 332.62591552734375, 153.9151441190388, 0.0, 0.0, 0.0, 1.0, 0.0]
    q.binning_x=0
    q.binning_y=0
    q.roi.x_offset=0
    q.roi.y_offset=0
    q.roi.height=0
    q.roi.width=0
    q.roi.do_rectify=False
    pub.publish(q)
    rate.sleep()
