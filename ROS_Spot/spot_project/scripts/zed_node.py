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

data = zedInfo()

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
angle = 0.0
distance = 0.0


def zed_callback(data):
	global numberOfFaces
	global centerX
	global centerY
	global rot_quat
	global faceSize
	global faceDetected
	global angle
	global msg
	
	
	bridge = CvBridge()
	frame = bridge.imgmsg_to_cv2(data)
	face, confidence = cv.detect_face(frame, enable_gpu = False) ## ** check confidence level before doing anything
	numberOfFaces = len(face)
	if numberOfFaces >= 1:   # Check confidence
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
		
		text1 = "{}: {:.2f}m".format('distance', distance)
		cv2.putText(frame, text1, (startX,startY), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255), 2)
		text = "{}: {:.2f}degrees".format('angle', angle)
		cv2.putText(frame, text, (startX,endY + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,), 2)
							
	angle = ((centerX-hResolution)/hResolution)* hFieldOfView
	if faceDetected == False:
		angle = 0.0
		
#	rospy.loginfo('angle: %s', angle)
	#rospy.loginfo('confidence: %s', confidence) # check confidence values and set a limit

		
	cv2.imshow('camera', frame) 
	cv2.waitKey(1)



def depth_callback(depth):
	global centerX
	global centerY
	global X
	global Y
	global distance
	
	bridge = CvBridge()
	depth_image = bridge.imgmsg_to_cv2(depth)
	depth_array = np.array(depth_image, dtype=np.float32)

	X = int(centerX)
	Y = int(centerY)
	
	distance = depth_array[Y,X]
	if faceDetected == False:
		distance = 0.0

	
rospy.init_node('zedNode', anonymous = True)
pub = rospy.Publisher('zedPublisher', zedInfo, queue_size = 10)	
sub1 = rospy.Subscriber('heat_video', Image, zed_callback, queue_size = 1,  buff_size=2**24)  
sub2 = rospy.Subscriber("/zedm/zed_node/depth/depth_registered", Image, depth_callback, queue_size = 1,  buff_size=2**24) ## subscribe to depth topic


rate = rospy.Rate(30)
cv2.destroyAllWindows()

while not rospy.is_shutdown():


	data.angle = angle
	data.distance = distance
	
	pub.publish(data.distance, data.angle, faceDetected)

	rate.sleep()



	
