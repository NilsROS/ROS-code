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

import numpy as np
import cv2

image = cv2.VideoCapture('yellowTest.jpg')
original = image.copy()
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower = np.array([22, 93, 0], dtype="uint8")
upper = np.array([45, 255, 255], dtype="uint8")
mask = cv2.inRange(image, lower, upper)

cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]

for c in cnts:
    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)

cv2.imshow('mask', mask)
cv2.imshow('original', original)
cv2.waitKey()

cap.release()

cv2.destroyAllWindows()

    
    
    
