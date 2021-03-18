#!/usr/bin/env python3

import rospy 
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import spot_ros_srvs.srv

from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2 
import cvlib as cv
import numpy as np
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import time
import math
import readchar
import sys
import os

from scipy.spatial.transform import Rotation
import datetime
from spot_ros_msgs.msg import zedInfo
from spot_ros_msgs.msg import KinematicState

distance = 0
rot_quat = [0,0,0,0]
ZedDistance = 0
ZedAngle = 0
faceDetectedByZed = 0
ThermalDistance = 0
ThermalAngle = 0
personDetectedByThermal = 0
RobotPosWorld = [0,0,0,0]
coordinates = [0,0,0,0]
speed = 0	


def robot_pos(msg):
	global RobotPosWorld
	global speed
	
	RobotPosWorld = msg.vision_tform_body
	speed = msg.velocity_of_body_in_vision
	#rospy.loginfo(RobotPosWorld)	
	
#	
#def thermal_callback(msg):
#	global ThermalAngle
#	global ThermalDistance
#	global personDetectedByThermal	
#	
#	ThermalAngle = msg.angle
#	ThermalDistance = msg.distance
#	personDetectedByThermal	= msg.personDetected	



def zed_callback(msg):
	global ZedAngle
	global ZedDistance
	global faceDetectedByZed
	
	ZedAngle = msg.angle
	ZedDistance = msg.distance
	faceDetectedByZed = msg.faceDetected
	


### Add sensor prioritation and the other sensor callbacks
def priority():
	global faceDetectedByZed
	global rot_quat
	global distance


	if faceDetectedByThermal == True:
		rot = Rotation.from_euler('xyz', [0, 0, ThermalAngle], degrees=True)
		rot_quat = rot.as_quat()
		distance = ThermalDistance

	elif faceDetectedByZed == True:
		rot = Rotation.from_euler('xyz', [0, 0, ZedAngle], degrees=True)
		rot_quat = rot.as_quat()
		distance = ZedDistance
		
	else:
		rot_quat = [0,0,0,0]
		distance = 0.0
		
#	rospy.loginfo(distance)
#	rospy.loginfo(rot_quat)
			
	
	
def trajectory_service(key):
	global rot_quat
	global distance
	global faceSize
	
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	
	if (personDetectedByThermal or faceDetectedByZed) and distance > 0.5:
	#if key in 't':
		#pose.position.x = distance
		pose.position.x = 2
		pose.position.y = 0
		pose.position.z = 0
		pose.orientation.x = 0 
		pose.orientation.y = 0 
		pose.orientation.z = rot_quat[2]
		pose.orientation.w = rot_quat[3]

		ps.poses.append(pose)		
		
	trajectory_srv_req.waypoints.poses = ps.poses

	try:
		rospy.wait_for_service("trajectory_cmd", timeout=5)
		trajectory_srv_pub(trajectory_srv_req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e, end='')
			
def global_trajectory_service(key):
	global coordinates
	
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	
	if key in 'g':
		pose.position.x = coordinates.translation.x
		pose.position.y = coordinates.translation.y
		pose.position.z = coordinates.translation.z
		pose.orientation.x = coordinates.rotation.x 
		pose.orientation.y = coordinates.rotation.y  
		pose.orientation.z = coordinates.rotation.z 
		pose.orientation.w = coordinates.rotation.w 

		ps.poses.append(pose)		
		
	global_trajectory_srv_req.globalwaypoints.poses = ps.poses

	try:
		rospy.wait_for_service("global_trajectory_cmd", timeout=5)
		global_trajectory_srv_pub(global_trajectory_srv_req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e, end='')


def orientation_service(key):
	global startOrientation
	global faceSize
	
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	
	if faceSize > 230 :
		pose.position.x = 0
		pose.position.y = 0
		pose.position.z = 0
		pose.orientation.x = 0 
		pose.orientation.y = 0.4
		pose.orientation.z = 0
		pose.orientation.w = 0
		
		ps.poses.append(pose)		
		
	orientation_srv_req.orientation.poses = ps.poses

	try:
		rospy.wait_for_service("orientation_cmd", timeout=5)
		orientation_srv_pub(orientation_srv_req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e, end='')
		
		
rospy.init_node('mainNode', anonymous = True)
sub1 = rospy.Subscriber('zedPublisher', zedInfo, zed_callback, queue_size = 10,  buff_size=2**24)
#sub3 = rospy.Subscriber('thermalPublisher', thermalInfo, thermal_callback, queue_size = 10,  buff_size=2**24)  ### Subscribe to Zed camera 2D image --- right_raw/image_raw_gray
#sub2 = rospy.Subscriber("zed_angle", zedInfo, angle_callback, queue_size = 1,  buff_size=2**24) ## subscribe to depth topic
sub2 = rospy.Subscriber('kinematic_state', KinematicState, robot_pos, queue_size =10)

rate = rospy.Rate(30)
cv2.destroyAllWindows()


# Define service proxies
self_right_srv_pub = rospy.ServiceProxy("self_right_cmd", spot_ros_srvs.srv.Stand)
stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
trajectory_srv_pub = rospy.ServiceProxy("trajectory_cmd", spot_ros_srvs.srv.Trajectory)
orientation_srv_pub = rospy.ServiceProxy("orientation_cmd", spot_ros_srvs.srv.Orientation)
global_trajectory_srv_pub = rospy.ServiceProxy("global_trajectory_cmd", spot_ros_srvs.srv.globalTrajectory)
 
 
# Define service requests
self_right_srv_req = spot_ros_srvs.srv.StandRequest()
stand_srv_req = spot_ros_srvs.srv.StandRequest()
trajectory_srv_req = spot_ros_srvs.srv.TrajectoryRequest()
orientation_srv_req = spot_ros_srvs.srv.OrientationRequest()
global_trajectory_srv_req = spot_ros_srvs.srv.globalTrajectoryRequest()
	

while not rospy.is_shutdown():
	
	key = readchar.readkey()
	if key == 'Q':
		sys.exit()
		
	if key in 's':
		coordinates = RobotPosWorld
		rospy.loginfo(coordinates)
		
	elif key in 't':
		priority()
		trajectory_service(key)	
		
		
	elif key in 'g':
		global_trajectory_service(key)	
		
	rate.sleep()	
	
