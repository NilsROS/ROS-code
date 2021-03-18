#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 

def callback(data):
 
  br = CvBridge()
  current_frame = br.imgmsg_to_cv2(data)
  
  face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades +'haarcascade_frontalface_default.xml') 
  #eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")

  #gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
  faces = face_cascade.detectMultiScale(current_frame, 1.3, 5)

  for (x,y,w,h) in faces:
     cv2.rectangle(current_frame,(x,y),(x+w,y+h),(255,0,0),2)
     #roi_gray = gray[y:y+h, x:x+w]
     roi_color = current_frame[y:y+h, x:x+w]
    # eyes = eye_cascade.detectMultiScale(roi_color)
  
 # for (ex,ey,ew,eh) in eyes:
  #      cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

  rospy.loginfo("receiving video frame")
   
  cv2.imshow("camera", current_frame) 
  cv2.waitKey(1)						
      
def receive_message():
  rospy.init_node('video_sub_py', anonymous=True)
  rospy.Subscriber('video_frames', Image, callback, queue_size=1, buff_size=2**24)
  rospy.spin()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
