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



NewX = 0
faceSize = 0
numberOfFaces = 0
faceDetected = False
hFieldOfView = 45 # divided by 2
hResolution = 336 #divided by 2
start = 0
centerX = 0
centerY = 0
rot_quat = [0,0,0,0]
startOrientation = 0
X = 0
Y = 0
angle = 0


def self_right_service(key):
    tf = geometry_msgs.msg.Transform()

    self_right_srv_req.body_pose.translation = tf.translation
    self_right_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("self_right_cmd", timeout=2.0)
        self_right_srv_pub(self_right_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def zed_callback(data):
	global numberOfFaces
	global centerX
	global centerY
	global rot_quat
	global faceSize
	global faceDetected
	global angle

	bridge = CvBridge()
	frame = bridge.imgmsg_to_cv2(data)
	face, confidence = cv.detect_face(frame, enable_gpu = True) ## ** check confidence level before doing anything
	numberOfFaces = len(face)
	if numberOfFaces >= 1:
		faceDetected = True
	else:
		faceDetected = False

	for idx, f in enumerate(face):
		(startX, startY) = f[0], f[1]
		(endX, endY) = f[2], f[3]
		cv2.rectangle(frame, (startX,startY), (endX,endY), (0,255,0), 2)
		Y = startY - 10 if startY - 10 > 10 else startY + 10
		centerX = ((startX + endX)/2) 
		centerY = ((startY + endY)/2) 
		faceSize = (startX + endX) - startX
								
	angle = ((centerX-hResolution)/hResolution)* hFieldOfView

	rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
	rot_quat = rot.as_quat()

	#rospy.loginfo('centerX: %s', centerX)
	#rospy.loginfo('confidence: %s', confidence)

	cv2.imshow('camera', frame) 
	cv2.waitKey(1)


def depth_callback(depth):
	global centerX
	global centerY
	global X
	global Y
	
	bridge = CvBridge()
	depth_image = bridge.imgmsg_to_cv2(depth)
	depth_array = np.array(depth_image, dtype=np.float32)

	X = int(centerX)
	Y = int(centerY)
	
	distance = depth_array[X,Y]
	rospy.loginfo(distance)

	
	
def trajectory_service(key):
	global rot_quat
	global startOrientation
	global faceSize
	
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	
	#if numberOfFaces >= 1 :
	if key == t :
		pose.position.x = 0
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
			


def orientation_service(key):
	global startOrientation
	global faceSize
	
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	
	#if faceSize > 230 :
	if key == 'o' and faceSize > 400:
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
		
		
rospy.init_node('camera_trajectory')
sub1 = rospy.Subscriber('heat_video', Image, zed_callback, queue_size = 1,  buff_size=2**24)  ### Subscribe to Zed camera 2D image --- right_raw/image_raw_gray
sub2 = rospy.Subscriber("/zedm/zed_node/depth/depth_registered", Image, depth_callback, queue_size = 1,  buff_size=2**24) ## subscribe to depth topic

rate = rospy.Rate(30)
cv2.destroyAllWindows()


# Define service proxies
self_right_srv_pub = rospy.ServiceProxy("self_right_cmd", spot_ros_srvs.srv.Stand)
stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
trajectory_srv_pub = rospy.ServiceProxy("trajectory_cmd", spot_ros_srvs.srv.Trajectory)
orientation_srv_pub = rospy.ServiceProxy("orientation_cmd", spot_ros_srvs.srv.Orientation)
 
 
# Define service requests
self_right_srv_req = spot_ros_srvs.srv.StandRequest()
stand_srv_req = spot_ros_srvs.srv.StandRequest()
trajectory_srv_req = spot_ros_srvs.srv.TrajectoryRequest()
orientation_srv_req = spot_ros_srvs.srv.OrientationRequest()
	

while not rospy.is_shutdown():

	key = readchar.readkey()
	# print('{}\rKey pressed: {}\r'.format(' '*int(columns), key), end="")

	if key=="Q":
		sys.exit()
		
	if key in "t":
		#trajectory_service(key)
		rospy.loginfo(faceSize)

	elif key in "o":
		#orientation_service(key)
		rospy.loginfo(angle)
	
	rate.sleep()	
#	while (faceDetected and faceSize < 400): ### Check this value
#		trajectory_service()
#		rospy.loginfo('traj')
#		
#	if faceSize >= 400:
#		orientation_service()
#		rospy.loginfo('orient')
		



		
