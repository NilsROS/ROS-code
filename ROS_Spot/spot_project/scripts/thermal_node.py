#!/usr/bin/env python3

import rospy 
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import spot_ros_srvs.srv
import time
import math
import readchar
import sys
import os
import cv2 
#import cvlib as cv
import numpy as np
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 

#from scipy.spatial.transform import Rotation
import datetime
from spot_ros_msgs.msg import zedInfo
from spot_ros_msgs.msg import thermalInfo


data = thermalInfo()

NewX = 0
faceSize = 0
numberOfFaces = 0
personDetected = False
hFieldOfView = 12 # divided by 2
hResolution = 336 #divided by 2
start = 0
centerX = 0
centerY = 0
rot_quat = [0,0,0,0]
startOrientation = 0
X = 0
Y = 0
angle = 0.0
distance = 0.0
# Used to convert between ROS and OpenCV images
bridge = CvBridge()

def thermal_callback(data):
	global numberOfFaces
	global centerX
	global centerY
	global rot_quat
	global faceSize
	global faceDetected
	global angle
	global msg
	global bridge 

	blob =0
	gray_min = 0
	gray_max =255
	# Convert back ROS Image message to OpenCV image.
	frame = bridge.imgmsg_to_cv2(data)
	# Convert form frame/image feed BGR to Gray feed. 
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# this is the mask for image feed. that makes everything to black and withe. 
	mask = cv2.inRange(gray,gray_min, gray_max)

	contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
	if len(contours) >=1:
		personDetected = True
	else:
		personDetected = False

	for cnt in contours:

		#area = cv2.contourArea(cnt) # This is if you whant to get ride of small anncesry blobs.
		#if area > 2000: # => withe the if stadenment 
		
		(x, y, w, h) = cv2.boundingRect(cnt)
		# This is for the form av the green markt area in image. 
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 10)
		# makrink of the image in center of x and y, NB OPS!!! this is put when the camera is on it's side 
		# so be wary about this and roteshen. 
		centerX = ((x + (x+w))/2) 
		centerY = ((y + (y+h))/2)
		X = int(centerX)
		Y = int(centerY)
		#rospy.loginfo(centerX)
		cv2.circle(frame,(X,Y),6,(0,255,0),-1)

		text = "x: " + str(X) + ", y: " + str(Y)
		cv2.putText(frame, text, (X - 10, Y - 10),
		cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 2)
		

		break
	# This is fore the roteshen and out put av Fluke/camera 
	image_out = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)					
	angle = ((centerX-hResolution)/hResolution)* hFieldOfView
	#if personDetected == False:
		#angle = 0.0
	
	# This is for the resize of the image. so width and height is sat = __ , __
	width ,height = 672 , 376
	imgResize = cv2.resize(image_out,(width,height)) # this is wear the image_out is transformd to a new size. 


	

	# This is to get the values of the angel of the seter image 
	rospy.loginfo('angle: %s', centerX) 
	#rospy.loginfo('confidence: %s', confidence) # check confidence values and set a limit

	# Display image to screen 
	cv2.imshow('camera', imgResize) 
	cv2.waitKey(1)

def depth_callback(depth):
	global centerX
	global centerY
	global X
	global Y
	global distance
	global bridge
	depth_image = bridge.imgmsg_to_cv2(depth)
	depth_array = np.array(depth_image, dtype=np.float32)

	X = int(centerX)
	Y = int(centerY)
	
	distance = depth_array[X,Y]
	rospy.loginfo(distance)


rospy.init_node('thermalNode', anonymous = True)
pub = rospy.Publisher('thermalPublisher', thermalInfo, queue_size = 10)	
sub1 = rospy.Subscriber('thermal_video', Image, thermal_callback, queue_size = 1,  buff_size=2**24)  ### Subscribe to Zed camera 2D image --- right_raw/image_raw_gray
sub2 = rospy.Subscriber("/zedm/zed_node/depth/depth_registered", Image, depth_callback, queue_size = 1,  buff_size=2**24) ## subscribe to depth topic


rate = rospy.Rate(30)
cv2.destroyAllWindows()

while not rospy.is_shutdown():


	data.angle = angle
	data.distance = distance
	
	pub.publish(data.distance, data.angle, personDetected)

	rate.sleep()



