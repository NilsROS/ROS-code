#!/usr/bin/env python3


import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import time
import threading 

duration=1


def message():
	pub = rospy.Publisher('thermal_video', Image, queue_size = 1)
	rospy.init_node('thermal', anonymous = True)
	
	
	rate = rospy.Rate(60)
	video = cv2.VideoCapture(0)
	bridge = CvBridge()
	while not rospy.is_shutdown():
		ret, image = video.read()
		
		if ret == True:
			cv2.imshow('camera', image)
			pub.publish(bridge.cv2_to_imgmsg(image))
			rospy.loginfo('publishing image')
	
		rate.sleep()
		
		
		

		

if __name__ == '__main__':
  try:
    message()
  except rospy.ROSInterruptException:
    pass
	

