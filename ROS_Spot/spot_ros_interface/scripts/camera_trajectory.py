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
#from spot_ros_interface import SpotInterface

#spot = SpotInterface()

NewX = 0
faceSize = 0
numberOfFaces = 0
faceDetected = False
hFieldOfView = 45 # divided by 2
hResolution = 360 #divided by 2
start = 0
centerX = 0
rot_quat = [0,0,0,0]

	
line='\u2500'
instructions="\n\
\u250C{} SPOT KEYBOARD TELEOP {}\u2510 \n\
\u2502                            \u2502\n\
\u2502     s - start trajectory   \u2502\n\
\u2502                            \u2502\n\
\u2502     r - Self-right         \u2502\n\
\u2502     j - Height up          \u2502\n\
\u2502     k - Height down        \u2502\n\
\u2502                            \u2502\n\
\u2502     SPACE - E-Stop (TODO)  \u2502\n\
\u2502     Q - Quit               \u2502\n\
\u2502                            \u2502\n\
\u2514{}\u2518\
".format(line*3,line*3,line*28)
# Get size of terminal window
rows, columns = os.popen('stty size', 'r').read().split()



def self_right_service(key):
    tf = geometry_msgs.msg.Transform()

    self_right_srv_req.body_pose.translation = tf.translation
    self_right_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("self_right_cmd", timeout=2.0)
        self_right_srv_pub(self_right_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

	
def stand_service(key):
    tf = geometry_msgs.msg.Transform()

    if key=='j':
        tf.translation.z = height_up
    elif key=='k':
        tf.translation.z = height_down

    stand_srv_req.body_pose.translation = tf.translation
    stand_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("stand_cmd", timeout=2.0)
        stand_srv_pub(stand_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def heat_callback(data):
	global numberOfFaces
	global centerX
	global rot_quat
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(data)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades +'haarcascade_frontalface_default.xml') 
	faces = face_cascade.detectMultiScale(gray, 1.1, 10) 
	numberOfFaces = len(faces)

	for (x,y,w,h) in faces:
		cv2.rectangle(gray, (x,y), (x+w, y+h), (0,255,0), 2)
		faceSize = ((x+w)-x)
		centerX = int(x+w/2) # center at about 325
		centerY = int(y+h/2) # center at about 220
		XMax = 530
		XMin = 95
		NewMax = -1
		NewMin = 1
		YMax = 370
		YMin = 95 
		NewMaxy = 0.5
		NewMiny = -0.5	
		NewY = (((centerY - YMin) * (NewMaxy - NewMiny)) / (YMax - YMin)) + NewMiny
		NewX = (((centerX - XMin) * (NewMax - NewMin)) / (XMax - XMin)) + NewMin

		if -0.2 < NewX < 0.2:
			NewX = 0
		if -0.2 < NewY < 0.2:
			NewY = 0
			
			
	angle = ((centerX-hResolution)/hResolution)* hFieldOfView
	
	rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)

	rot_quat = rot.as_quat()
    
#	rospy.loginfo("angle:%s", angle)
#	rospy.loginfo("qx:%s", rot_quat)


	cv2.imshow('camera', gray) 
	cv2.waitKey(1)
	
	
def trajectory_service(key):
	global rot_quat
	ps = geometry_msgs.msg.PoseArray()  
	pose = geometry_msgs.msg.Pose()	
	
	if key == 'f':
		pose.position.x = 0
		pose.position.y = 0
		pose.position.z = 0
		pose.orientation.x = 0#rot_quat[0]
		pose.orientation.y =0.2 #rot_quat[1]
		pose.orientation.z =0 #rot_quat[2]
		pose.orientation.w = 1#rot_quat[3]
		rospy.loginfo_once("rot%s", rot_quat[2])
		rospy.loginfo_once("rot%s", rot_quat[3])
		ps.poses.append(pose)		
		

	trajectory_srv_req.waypoints.poses = ps.poses

	try:
		rospy.wait_for_service("trajectory_cmd", timeout=5)
		trajectory_srv_pub(trajectory_srv_req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e, end='')
	#start = 0	




rospy.init_node('camera_trajectory')
sub = rospy.Subscriber('heat_video', Image, heat_callback, queue_size = 1,  buff_size=2**24)
rate = rospy.Rate(30)
cv2.destroyAllWindows()

# Define service proxies
self_right_srv_pub = rospy.ServiceProxy("self_right_cmd", spot_ros_srvs.srv.Stand)
stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
trajectory_srv_pub = rospy.ServiceProxy("trajectory_cmd", spot_ros_srvs.srv.Trajectory)
 
# Define service requests
self_right_srv_req = spot_ros_srvs.srv.StandRequest()
stand_srv_req = spot_ros_srvs.srv.StandRequest()
trajectory_srv_req = spot_ros_srvs.srv.TrajectoryRequest()

	

print(instructions)


while not rospy.is_shutdown():
	while(1):
	
		
		rospy.loginfo(1)
		
		key = readchar.readkey()
		#print('{}\rKey pressed: {}\r'.format(' '*int(columns), start), end="")
		rospy.loginfo(2)
		if key=="Q":
		    sys.exit()
		rospy.loginfo(3)
		    

		if key in 'jk':
		    stand_service(key)
	   # rospy.loginfo(4)

		elif key in 'r':
		    self_right_service(key)
		    
		elif key in 'f':
			trajectory_service(key) 
		
		   
		rate.sleep()
		#rospy.spin()

		
