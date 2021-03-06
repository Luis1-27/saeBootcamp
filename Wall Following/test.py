#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270           # Hokuyo 10LX has 270 degree scan.
DISTANCE_THRESHOLD = 1      # Distance threshold before collision (m)
VELOCITY = 0.5              # Maximum Velocity of the vehicle
TIME_THRESHOLD = 1          # Time threshold before collision (s)
STEERING_ANGLE = 0          # Steering angle is uncontrolled
WITHIN_THRESHOLD = False    # To track the vehicle status in previous execution

# P-Controller Parameters
kp_dist = 0.3
kp_ttc = 0.1

dist_error = 0.0
time_error = 0.0

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

def dist_control(distance):
	global kp_dist
	global VELOCITY
	global DISTANCE_THRESHOLD 
	global STEERING_ANGLE
	
	# TO-DO: Calculate Distance to Collision Error
	# ---
	velocity = VELOCITY
	dist_error = distance - DISTANCE_THRESHOLD
	velocity = dist_error*kp_dist
	if dist_error < 0.01:  # If the error falls within a tolerance then stop the car
		velocity = 0
	if distance == 0:  # To avoid car being stopped
		velocity = VELOCITY
	# ---

	print("Distance before collision is = ", distance)
	print("Vehicle velocity= ", velocity)

	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = STEERING_ANGLE
	pub.publish(msg)

def TTC_control(distance):
	global kp_ttc
	global TIME_THRESHOLD
	global VELOCITY
	global STEERING_ANGLE
	global speed
	global WITHIN_THRESHOLD
	
	# TO-DO: Calculate Time To Collision Error
	# ---
	time = distance/speed
	if distance == 0:  # To avoid car being stopped
		time = 15  # Providing some big value to keep car moving
	
	time_error = time - TIME_THRESHOLD
	
	if WITHIN_THRESHOLD: # Storing the previous state of vehicle within time threshold to avoid noisy behavior
		velocity = 0
	else:
		if time <= TIME_THRESHOLD and time_error < 0.3:  # If time to collision is within the threshold and the error falls within the tolerance then stop the car
			WITHIN_THRESHOLD = True
			velocity = 0
		elif time > 5:  # If time to collision is more than 5 seconds then go at full speed
			velocity = VELOCITY
		else:  # If time to collision is within 5 seconds and time threshold then start braking
			velocity = time_error*kp_ttc

	# ---
	print("Time error is = ", time_error)	
	print("Time to collision in seconds is = ", time)
	print("Vehicle velocity = ", velocity)

	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = STEERING_ANGLE
	pub.publish(msg)

def get_index(angle, data):
	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# converted the angle to radians and did an offset with angle minimum to give an angle range from zero to angle max. 
	req_angle = math.radians(angle) - data.angle_min
	
	#Calculated the range indices per radian.
	indices_per_radian = len(data.ranges)/(data.angle_max - data.angle_min)
	
	#Multiply the indices per radian with the modified angle. This will give a ratio and flooring it to nearest integer will give the index corresponding to the angle requested.
	index = math.floor(req_angle*indices_per_radian)
	return index

	# ---
	
# Use this function to find the average distance of a range of points directly in front of the vehicle.
def get_distance(data): 
	global ANGLE_RANGE
	
	angle_front = 0   # Range of angle in the front of the vehicle we want to observe
	avg_dist = 0
	offset_indices = 50
	dist_list = []  
	
	# Get the corresponding list of indices for given range of angles
	index_front = get_index(angle_front, data)
	
	for i in range(index_front-offset_indices, index_front+offset_indices, 1):
		if not math.isinf(data.ranges[index_front]):
			dist_list.append(data.ranges[index_front])

	# TO-DO: Find the avg range distance
	# ---
	if dist_list:
		avg_dist = sum(dist_list)/len(dist_list)
	else:
		avg_dist = 0
	# ---
	print("Average Distance = ", avg_dist)

	return avg_dist

def callback(laser_data):

	# TO-DO: Complete the Callback. Get the distance and input it into the controller
	# ---
	dist_control(get_distance(laser_data))
	##TTC_control(get_distance(laser_data))
	# ---

def speed_callback(data):
	global speed
	speed = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2 + data.twist.twist.linear.z**2)
	return None


if __name__ == '__main__':
	print("AEB started")
	rospy.init_node('aeb',anonymous = True)
	#laser_sub = message_filters.Subscriber('/scan', LaserScan)
	#odom_sub = message_filters.Subscriber('/ground_truth', Odometry)
	#ackdrive_sub = message_filters.Subscriber('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped)
	#ts = message_filters.TimeSynchronizer([laser_sub, odom_sub], 5)
	#ts.registerCallback(callback)
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.Subscriber("/vesc/odom",Odometry,speed_callback)
	rospy.spin()
		
