#!/usr/bin/env python3


import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np
import math
import time

from scipy.spatial.transform import Rotation
import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import threading
from datetime import datetime

NewX = 0
faceSize = 0
numberOfFaces = 0
faceDetected = False
hFieldOfView = 45 # divided by 2
hResolution = 360 #divided by 2
centerX = 0
rot_quat =[0,0,0,0] 


def heat_callback(data):
	global centerX
	global numberOfFaces
	global rot_quat
	global faceSize
	
	
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(data)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades +'haarcascade_eye.xml') 
	faces = face_cascade.detectMultiScale(gray, 1.2, 10) 

	numberOfFaces = len(faces)
	for (x,y,w,h) in faces:
		cv2.rectangle(gray, (x,y), (x+w, y+h), (0,255,0), 2)
		faceSize = ((x+w)-x)
		centerX = int(x+w/2) # Center of face bounding box
		centerY = int(y+h/2) 
	
	cv2.imshow('camera', gray)

	cv2.waitKey(1)
		
#		XMax = 530
#		XMin = 95
#		NewMax = -1
#		NewMin = 1
#		YMax = 370
#		YMin = 95 
#		NewMaxy = 0.5
#		NewMiny = -0.5	
#		NewY = (((centerY - YMin) * (NewMaxy - NewMiny)) / (YMax - YMin)) + NewMiny
#		NewX = (((centerX - XMin) * (NewMax - NewMin)) / (XMax - XMin)) + NewMin

#		if -0.2 < NewX < 0.2:
#			NewX = 0
#		if -0.2 < NewY < 0.2:
#			NewY = 0

	#rospy.loginfo("center%s", centerX)		
#	angle = ((centerX-hResolution)/hResolution)* hFieldOfView

#	rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
#	rot_quat = rot.as_quat()
	

rospy.init_node('heat_subscriber', anonymous = True)
sub = rospy.Subscriber('heat_video', Image, heat_callback, queue_size = 1,  buff_size=2**24)
rate = rospy.Rate(60)			
cv2.destroyAllWindows()

while not rospy.is_shutdown():
	
	rate.sleep()


