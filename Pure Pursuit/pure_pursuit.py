#!/usr/bin/env python3
# Imports
import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg
from rospkg import RosPack
from nav_msgs.msg import Odometry
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import matplotlib.pyplot as plt
from matplotlib import patches

# GLOBAL VARIABLES 
xc = 0
yc = 0
yaw = 0 
idx = 0
waypoints = []

# CAR VARIABLES
LOOKAHEAD = 1.2
WB = 0.3421

#PROGRAM VARIABLES
pure_pursuit_flag = True
show_animation = True

drive_pub = drive_param()

def read_points():
	# CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
	r = rospkg.RosPack()
	package_path=r.get_path('pure_pursuit')
	file_name='new_file.csv'
	file_path=package_path+'/waypoints/'+file_name
	with open(file_path) as f:
		path_points = [tuple(line) for line in csv.reader(f)]
	return path_points

def pose_callback(data):	
	global xc,yc,yaw
	xc = data.pose.pose.position.x
	yc = data.pose.pose.position.y
	
	# Convert Quaternions to Eulers
	qx=data.pose.pose.orientation.x
	qy=data.pose.pose.orientation.y
	qz=data.pose.pose.orientation.z
	qw=data.pose.pose.orientation.w
	quaternion = (qx,qy,qz,qw)
	euler = euler_from_quaternion(quaternion)
	yaw = euler[2]

def find_distance(x1,y1):
	global xc,yc,yaw,waypoints
	distance = math.sqrt((x1-xc)**2 + (y1-yc)**2)
	return distance

def find_distance_index_based(idx):
	global xc,yc,yaw,waypoints
	x1=float(waypoints[idx][0]);y1=float(waypoints[idx][1])
	distance = math.sqrt((x1-xc)**2 + (y1-yc)**2)
	return distance

def interpolate(idx):
	global xc,yc,yaw,waypoints,LOOKAHEAD
	x1=float(waypoints[idx][0])
	y1=float(waypoints[idx][1])
	x2=float(waypoints[idx+1][0])
	y2=float(waypoints[idx+1][1])


	###### INTERPOLATION ALGEBRA PART ###########
	## TODO: Find the interpolation. Try catch will help catch the case when there is no solution
	
	# define slope
	m = ((y2-y1)/(x2-x1))
	
	# defining parts for quadratic equation
	a = (m**2 + 1)
	
	b = ((-2*x1*m**2) + (2*m*y1) - (2*m*yc) - (2*xc))
	
	c = ((-LOOKAHEAD**2)+((m**2)*(x1**2))-(2*m*x1*y1)+(2*m*x1*yc)+(xc**2)+(y1**2)-(2*y1*yc)+(yc**2))


	try :
		flag = "interpolated pt" # When there are 1 or 2 interpolation points.
		
		# Solving the x and y roots using the quadratic pieces above
		xroot1 = (-b + (math.sqrt((b**2)-4*a*c)))/(2*a)
		yroot1 = ((m*xroot1)-(m*x1-y1)) 
		xroot2 = (-b - (math.sqrt((b**2)-4*a*c)))/(2*a)
		yroot2 = ((m*xroot2)-(m*x1-y1))


		# Resolve between two possible conflicting solutions
		# Finding the distances between the points and the first waypoint. 
		distance1 = math.sqrt((xroot1-x1)**2 + (yroot1-y1)**2) 
		distance2 = math.sqrt((xroot2-x1)**2 + (yroot2-y1)**2)
		x = xroot1
		y = yroot1
		
		# If the first root is farther away from the first waypoint, lets use the second root
		if distance1 > distance2:
		   x=xroot2
		   y=yroot2


	except ValueError:
		flag = "imaginary soln" # When there is no solution. Set x and y simply to the next waypoint (x2 y2)
		x = x2
		y = y2

	return x,y,flag

def find_nearest_waypoint(): 
	global xc,yc,yaw,waypoints
	min_distance = 100
	for point in waypoints:
		x1=float(point[0]) 
		y1=float(point[1])
		distance = math.sqrt((x1-xc)**2 + (y1-yc)**2)
		if distance < min_distance:
			min_distance = distance
			nearest_idx = waypoints.index(point)
	return nearest_idx


def idx_close_to_lookahead(idx): 
	global LOOKAHEAD
	while find_distance_index_based(idx) < LOOKAHEAD:
		idx += 1 
	return idx-1 

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
	"""
	Plot arrow
	"""
	if not isinstance(x, float):
		for ix, iy, iyaw in zip(x, y, yaw):
			plot_arrow(ix, iy, iyaw)
	else:
		plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
				  fc=fc, ec=ec, head_width=width, head_length=width)
		plt.plot(x, y)
		patches.Rectangle((xc,yc), 0.35,0.2)

def follow_waypoints():
	global xc,yc,yaw,waypoints,idx,LOOKAHEAD,WB,pure_pursuit_flag,show_animation
	rospy.Subscriber("/vesc/odom", Odometry, pose_callback) 
	pub = rospy.Publisher("drive_parameters", drive_param, queue_size=1) # velocity and angle

	#PURE PURSUIT CODE 
	cx = []; cy = []
	for point in waypoints:
		cx.append(float(point[0]))
		cy.append(float(point[1]))
	
	try:
		while pure_pursuit_flag:
			nearest_idx = find_nearest_waypoint()
			idx_near_lookahead = idx_close_to_lookahead(nearest_idx)
			target_x, target_y, flag = interpolate(idx_near_lookahead)
		
			while flag == "imaginary soln": # When there is no interpolation 
				
				
				# go-to-goal controller
				alpha = math.atan2(target_y - yc,target_x - xc) - yaw
				distance_error = find_distance_index_based(nearest_idx + 1)
				
				kd = 1
				ka = 1


				# publish messages
				drive_pub.velocity = np.clip(kd*distance_error, 0, 0.2)
				drive_pub.angle = np.clip(ka*alpha, -0.51, 0.51)
				
				# Debug info for Controller 2
				#print("solution :", flag)
				#print("nearest_index",nearest_idx)
				#print("nearest_index @ ", d1)
				#print("next index @ ", d2)
				#print("-------------")
				#if d1 > d2: 
				#	print("next index closer than previously nearest index")
				#	break

			# pure pursuit controller
			alpha = math.atan2(target_y-yc, target_x-xc) - yaw
			delta = math.atan2(2*math.sin(alpha)*WB,LOOKAHEAD)
			delta = np.clip(delta,-0.51,0.51)
                           
			# publish messages
			drive_pub.velocity = 1
			drive_pub.angle = delta
			pub.publish(drive_pub)
			time.sleep(0.1)


			if show_animation:
				plt.cla()
				# for stopping simulation with the esc key.
				plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
				plot_arrow(xc, yc, yaw)
				plt.plot(cx, cy, "-r", label="course")
				plt.plot(xc, yc, "-b", label="trajectory")
				plt.plot(target_x, target_y, "xg", label="target")
				plt.axis("equal")
				plt.grid(True)
				plt.title("Pure Pursuit Control" + str(1))
				plt.pause(0.001)
	
			# prints for debugging
			print("pure pursuit flag:",pure_pursuit_flag)
			print("solution :", flag)
			print("nearest_waypoint", nearest_idx)
			print("idx_near_lookahead", idx_near_lookahead)
			print("vehicle x:", xc,"vehicle y:", yc)
			print("following --> target_x :",target_x, "target_y :", target_y)
			print("steering angle :", delta)
			print("----------------------")
	except IndexError:
		print("PURE PURSUIT COMPLETE --> COMPLETED ALL WAYPOINTS")
		

if __name__=='__main__':
	rospy.init_node('pure_pursuit')
	r = rospy.Rate(10)
	print("RUNNING PURE-PURSUIT CODE.... \n\n")
	time.sleep(2)
	waypoints = read_points()
	follow_waypoints()
