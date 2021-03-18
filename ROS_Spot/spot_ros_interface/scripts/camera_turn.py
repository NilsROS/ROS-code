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

import time
import math
import readchar
import sys
import os



line='\u2500'
instructions="\n\
\u250C{} SPOT KEYBOARD TELEOP {}\u2510 \n\
\u2502                            \u2502\n\
\u2502                            \u2502\n\
\u2502     s - start camMode      \u2502\n\
\u2502     r - Self-right         \u2502\n\
\u2502                            \u2502\n\
\u2502                            \u2502\n\
\u2502                            \u2502\n\
\u2502     SPACE - E-Stop (TODO)  \u2502\n\
\u2502     Q - Quit               \u2502\n\
\u2502                            \u2502\n\
\u2514{}\u2518\
".format(line*3,line*3,line*28)

# Get size of terminal window
rows, columns = os.popen('stty size', 'r').read().split()

# Define velocity at which Spot will move
lin_vel = 0.5 # m/s

NewX = 0
NewY = 0
start = 0

def heat_callback(data):
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(data)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades +'haarcascade_frontalface_default.xml') 
	faces = face_cascade.detectMultiScale(gray, 1.1, 5) # image, scale factor: model with fixed size during training - so this size is detected in images- but by resizing the input image a large face can become a smaller one making it detectable to the algorithm , minimum number of neighbours.
	for (x,y,w,h) in faces:
		cv2.rectangle(gray, (x,y), (x+w, y+h), (0,255,0), 2) #image, starting point for rectange-x and y, ending point for rectange-x and y, BGR- here B-blue, thickness
		centerX = int(x+w/2) #center at about 325
		centerY = int(y+h/2) # center at about 220
		XMax = 530
		XMin = 95 
		NewMax = 1
		NewMin = -1	
		global NewX
		YMax = 370
		YMin = 95 
		NewMaxy = 0.5
		NewMiny = -0.5	
		global NewY
		NewY = (((centerY - YMin) * (NewMaxy - NewMiny)) / YMax - YMin)) + NewMiny
		
		NewX = (((centerX - XMin) * (NewMax - NewMin)) / (XMax - XMin)) + NewMin
	
	if -0.15 < NewX < 0.15:
		NewX = 0
		
	if -0.15 < NewY < 0.15:
		NewY = 0
			
	#rospy.loginfo("receiving video frame %s", NewX)
	cv2.imshow('camera', gray)  
	cv2.waitKey(1) #how long to show a picture before moving on, 1ms


def self_right_service(key):
    tf = geometry_msgs.msg.Transform()

    self_right_srv_req.body_pose.translation = tf.translation
    self_right_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("self_right_cmd", timeout=2.0)
        self_right_srv_pub(self_right_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def stand_service():
    tf = geometry_msgs.msg.Transform()
    global NewY
    tf.translation.z = NewY
    stand_srv_req.body_pose.translation = tf.translation
   
    try:
        rospy.wait_for_service("stand_cmd", timeout=2.0)
        stand_srv_pub(stand_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
	

def vel_service():
    twist = geometry_msgs.msg.Twist()
    global NewX
    twist.angular.z = NewX
    vel_srv_req.velocity.angular = twist.angular

    try:
        rospy.wait_for_service("velocity_cmd", timeout=2.0)
        vel_srv_pub(vel_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e, end='')


rospy.init_node('keyboard_teleop')
sub = rospy.Subscriber('heat_video', Image, heat_callback, queue_size = 1,  buff_size=2**24)
rate = rospy.Rate(60)
cv2.destroyAllWindows()

# Define service proxies
self_right_srv_pub = rospy.ServiceProxy("self_right_cmd", spot_ros_srvs.srv.Stand)
stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
vel_srv_pub = rospy.ServiceProxy("velocity_cmd", spot_ros_srvs.srv.Velocity)

# Define service requests
self_right_srv_req = spot_ros_srvs.srv.StandRequest()
stand_srv_req = spot_ros_srvs.srv.StandRequest()
vel_srv_req = spot_ros_srvs.srv.VelocityRequest()

print(instructions)
while not rospy.is_shutdown():
    key = readchar.readkey()
    print('{}\rKey pressed: {}\r'.format(' '*int(columns), key), end="")
      
    if key=="Q":
        sys.exit()
        
    if key == 's':
    	start = 1    
      
    elif key in 'r':
        self_right_service(key)
        
    if start == 1:    
    	vel_service()
    	stand_service()


    rate.sleep()


