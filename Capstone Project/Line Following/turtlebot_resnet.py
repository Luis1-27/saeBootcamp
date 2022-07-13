#!/usr/bin/env python3

import sys
import rospy
import roslib
import numpy as np
import cv2
import tensorflow as tf
import rosnode
from tensorflow.keras.preprocessing import image
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input, decode_predictions
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from object_recognition.msg import Predictor

time.sleep(31)

ourPred = Predictor()

# Create a bridge between ROS and OpenCV
bridge = CvBridge()
# These settings are for GPU options. They are technically not used here.
# The computer used does not have a GPU. 
GPU_OPTIONS = tf.compat.v1.GPUOptions(allow_growth=True)
CONFIG = tf.compat.v1.ConfigProto(inter_op_parallelism_threads=1,intra_op_parallelism_threads=1,gpu_options=GPU_OPTIONS)
CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.5

# Create tensorflow and keras sessions
sess = tf.compat.v1.Session(config=CONFIG)
tf.compat.v1.keras.backend.set_session(sess)
# Call our trained model
model = ResNet50(weights='imagenet')

# Create graph with target size for image
graph = tf.compat.v1.get_default_graph()
target_size = (224, 224)

# variable for publisher
global counter
counter = 0

#variables for apriltag
tagData = ''
tag0cnt = 0


def apriltag_callback(data):
    
    global tagData
    tagData = data

# Defining the callback from the subscriber
def callback(image_msg):

    global sess
    global graph
    global model
    global counter
    global tagData
    global tag0cnt
    
    
    #print(tagData.data)
    
    if tagData.data == "id: [0]":
       tag0cnt = tag0cnt + 1

       
    #if tag 1 or 2 are seen at least once then reset tag 0 counter and stop line following
    elif (tagData.data == "id: [1]") or (tagData.data == "id: [2]"):
       tag0cnt = 0
    if tag0cnt >= 1:
      
      try:    
       
          #Convert image to OpenCV Image
          cv_image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    
          # Resize image using the target size above
          cv_image = cv2.resize(cv_image, target_size)
          # Convert to array
          np_image = np.asarray(cv_image)
          # Expand the dimensions 
          np_image = np.expand_dims(np_image, axis=0)
          # Convert array to float type
          np_image = np_image.astype(float)
          # Preprocess the array
          np_image = preprocess_input(np_image) 
    
          # Start predicting using the model
          preds = model.predict(np_image)
          # Decode the prediction string
          pred_string = decode_predictions(preds, top=1)
          #Print the prediction string
          print(pred_string)

          label = pred_string[0][0][1] 
          if label == 'street_sign':
            counter = counter+1

          ourPred.header.stamp = rospy.Time.now()
          ourPred.label = label
          ourPred.score = counter
          predPub.publish(ourPred)
          
          if counter == 34:
              rospy.signal_shutdown("finished with object rec")
              
      except CvBridgeError as e:
          print(e)
          
    else:
      tag0cnt = 0
      pass
         
# Initialize node
rospy.init_node('classify', anonymous=True)
# Initialize subscriber
rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback, queue_size = 1, buff_size = 16777216)
rospy.Subscriber("/chatter", String, apriltag_callback, queue_size = 1)
predPub = rospy.Publisher("object_recognition", Predictor, queue_size = 1)

while not rospy.is_shutdown():
    rospy.spin()
