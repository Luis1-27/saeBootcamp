#!/usr/bin/env python3

import rospy
import time
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270 				# Hokuyo 10LX has 270 degrees scan
DISTANCE_THRESHOLD = 0.5 	# (m)
VELOCITY = 1.0				# meters per second
FREQUENCY = 0.1 # Hz
# Controller parameters
kp = 0.7
ki = 0.0
kd = 0.0
kpv = 0.9

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

# Other global variables
error = 0.0
prev_er = 0.0
int_prev =  0.0


def control(error):
	global kp
	global kd
	global ki
	global kpv
	global prev_er
	global int_prev
	global VELOCITY
	global velocity
	
	time = 1/FREQUENCY
	integral = int_prev + error*time
	deriv = (error - prev_er)/time
	
	velocity = VELOCITY
	# TO-DO: Implement controller
	# ---
	steering_angle_c = kp*error + ki*integral + kd*deriv
	# ---
	# Calculating velocity compensation. Uncomment this when other parts of the code works. Absolute is used so that 
	# irrespective of steering angle direction, we want the velocity to reduce. If the steering angle is in a negative
	# direction but abs is NOT used then final velocity will be greater than the vehicle max.
	velocity = VELOCITY - abs(kpv*steering_angle_c)
	
	# Set maximum thresholds for steering angles
	if steering_angle_c > 0.5:
		steering_angle = 0.5
	elif steering_angle_c < -0.5:
		steering_angle = -0.5
	else:
		steering_angle = steering_angle_c
	
	prev_er = error
	int_prev = integral
	

	print ("Steering Angle is = %f" % steering_angle )
	print("Commanding Velocity is = %f" % velocity)

	# TO-DO: Publish the message
	# ---
	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = steering_angle
	pub.publish(msg)
	# ---
	return velocity

def get_index(angle, data):
	# 	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# ---
	req_angle = math.radians(angle) - data.angle_min
	
	#Calculated the range of indices per radian.
	indices_per_radian = len(data.ranges)/(data.angle_max - data.angle_min)
	
	#Multiply the indices per radian with the modified angle. This will give a ratio and flooring it to 
	# nearest integer will give the index corresponding to the angle requested.
	# ---
	index = math.floor(req_angle*indices_per_radian)
	return index
	
def avg_distance(index, data):
	offset_indices = 30
	dist_list = []
	for i in range(index - offset_indices, index + offset_indices, 1):
		if not math.isinf(data.ranges[i]):
			dist_list.append(data.ranges[i])
			
	# TO-DO: Find the avg range distance
	if dist_list:
		return sum(dist_list)/len(dist_list)
	else:
		return 0

def distance(angle_side, angle_lookahead, data, direction):
	global ANGLE_RANGE
	global DISTANCE_THRESHOLD

	# TO-DO: Find index of the two rays, and calculate a, b, alpha and theta. Find the actual distance from the right wall.
	# ---
	index_side = get_index(angle_side, data)
	index_side_lookahead = get_index(angle_lookahead, data)
	
	distance_a = avg_distance(index_side_lookahead, data)
	distance_b = avg_distance(index_side, data)
	
	theta = angle_side-angle_lookahead
	
	numerator_alpha = (distance_a* math.cos(math.radians(theta))) - distance_b
	denominator_alpha = distance_a*math.sin(math.radians(theta))
	
	alpha = math.atan(numerator_alpha/denominator_alpha)
	#alpha_d = math.degrees(alpha)
	dist_pred = 0.1/FREQUENCY
	distance_r = distance_b*math.cos(alpha) #+ dist_pred * math.sin(alpha)
	
	# ---
	#print("dist_pred: %f" % dist_pred)
	#print("Alpha: %f" % alpha_d)
	print ("Distance from {direction} wall : %f" % distance_r)

	# Calculate error
	error = DISTANCE_THRESHOLD - distance_r
	
	print("Distance Error from {direction} wall: %f" % error)

	return error, distance_r


def follow_center(angle_right,angle_lookahead_right, data):

	angle_lookahead_left = abs(angle_right)
	angle_left = abs(angle_lookahead_right) 

	er, dr = distance(angle_right, angle_lookahead_right, data, "right")
	el, dl = distance(angle_left, angle_lookahead_left, data, "left" )

	# Find Centerline error
	# ---
	centerline_error = er - el
	# ---

	print("Centerline error = %f " % centerline_error)

	return centerline_error

def callback(data):

	# Pick two rays at two angles
	angle_right = -90
	angle_lookahead = -30

	# To follow right wall
	#er, dr = distance(angle_right,angle_lookahead, data, 'right')

	# To follow the centerline
	ec = follow_center(angle_right,angle_lookahead, data)

	control(ec)
	
def speed_callback(data):
	global speed
	speed = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2 + data.twist.twist.linear.z**2)
	return None

if __name__ == '__main__':
	print("Wall following started")
	rospy.init_node('wall_following',anonymous = True)

	# TO-DO: Implement the publishers and subscribers
	# ---
	rate = rospy.Rate(FREQUENCY)
	while not rospy.is_shutdown():
		rospy.Subscriber("/scan",LaserScan,callback)
		rospy.Subscriber("/vesc/odom",Odometry,speed_callback)
		rate.sleep()
	# ---
		rospy.spin()
