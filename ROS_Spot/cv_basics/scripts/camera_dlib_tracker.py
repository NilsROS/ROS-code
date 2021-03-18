#!/usr/bin/env python3


import rospy 
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import spot_ros_srvs.srv

from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
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
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import dlib


NewX = 0
faceSize = 0
numberOfFaces = 0
faceDetected = False
hFieldOfView = 45 # divided by 2
hResolution = 360 #divided by 2
start = 0
centerX = 0
rot_quat = [0,0,0,0]
startOrientation = 0
confirmFace = 0



def self_right_service(key):
    tf = geometry_msgs.msg.Transform()

    self_right_srv_req.body_pose.translation = tf.translation
    self_right_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("self_right_cmd", timeout=2.0)
        self_right_srv_pub(self_right_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)




def heat_callback(data):
	global numberOfFaces
	global centerX
	global rot_quat
	global faceSize
	global faceDetected
	global confirmFace
	detector = dlib.get_frontal_face_detector()
	old_faces = []	
	
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(data)


	imageResize = cv2.resize(image, (0, 0), fx=0.2, fy=0.2)

	faces = detector(imageResize, 1)
	if len(old_faces) < len(faces):
	    old_faces = []
	    for face in faces:
	        tracker = dlib.correlation_tracker()
	        tracker.start_track(imageResize, face)
	        old_faces.append(tracker)
	else:
	    for i, tracker in enumerate(old_faces):
	        quality = tracker.update(imageResize)
	        if quality > 7:
	            pos = tracker.get_position()
	            pos = dlib.rectangle(
	                int(pos.left()),
	                int(pos.top()),
	                int(pos.right()),
	                int(pos.bottom()),
	            )
	            cv2.rectangle(imageResize, (pos.left(), pos.top()), (pos.right(), pos.bottom()),
	                          (100, 200, 100))
	        else:
	            old_faces.pop(i)

			
	angle = ((centerX-hResolution)/hResolution)* hFieldOfView
	
	rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
	rot_quat = rot.as_quat()


	cv2.imshow('camera', imageResize) 
	cv2.waitKey(1)
	
	
def trajectory_service():
	global rot_quat
	global startOrientation
	global faceSize
	
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	rospy.logonce('traj')
	if numberOfFaces >= 1 :
		pose.position.x = 0.5
		pose.position.y = 0
		pose.position.z = 0
		pose.orientation.x = 0 #rot_quat[0]
		pose.orientation.y = 0 #rot_quat[1]
		pose.orientation.z = rot_quat[2]
		pose.orientation.w = rot_quat[3]
#		rospy.loginfo_once("rot%s", rot_quat[2])
#		rospy.loginfo_once("rot%s", rot_quat[3])
		ps.poses.append(pose)		
		

	trajectory_srv_req.waypoints.poses = ps.poses

	try:
		rospy.wait_for_service("trajectory_cmd", timeout=5)
		trajectory_srv_pub(trajectory_srv_req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e, end='')
			

def orientation_service():
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
		
		

rospy.init_node('camera_trajectory')
sub = rospy.Subscriber('heat_video', Image, heat_callback, queue_size = 1,  buff_size=2**24)
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
	
		
	while numberOfFaces >= 1: ### Check this value
		trajectory_service()
		
	if faceSize >= 230:
		orientation_service()
				
		   
		rate.sleep()
		#rospy.spin()

