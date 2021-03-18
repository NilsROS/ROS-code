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
import cv2 
import imutils 
import cvlib as cv

NewX = 0
faceSize = 0
numberOfFaces = 0
faceDetected = False
hFieldOfView = 45 # divided by 2
hResolution = 360 #divided by 2
centerX = 0
rot_quat =[0,0,0,0] 
startX = 0
startY = 0
centerX = 0
centerY = 0

def heat_callback(data):
    global startX
    global startY
    global centerX
    global centerY
    
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data)

    face, confidence = cv.detect_face(frame)
    for idx, f in enumerate(face):
        (startX, startY) = f[0], f[1]
        (endX, endY) = f[2], f[3]
        cv2.rectangle(frame, (startX,startY), (endX,endY), (0,255,0), 2)
        #text = "{:.2f}%".format(confidence[idx] * 100)
        Y = startY - 10 if startY - 10 > 10 else startY + 10
        centerX = ((startX + endX)/2) # X from 0-640
        centerY = ((startY + endY)/2) #Y from 0-780
        

    #cv2.putText(frame, text, (startX,Y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                #(0,255,0), 2)
    rospy.loginfo('centerX%s', centerX)
    cv2.imshow("Real-time face detection", frame)
    cv2.waitKey(1)


rospy.init_node('heat_subscriber', anonymous = True)
rospy.Subscriber('heat_video', Image, heat_callback, queue_size = 1,  buff_size=2**24)
rate = rospy.Rate(60)			
cv2.destroyAllWindows()

while not rospy.is_shutdown():
	
	rate.sleep()

