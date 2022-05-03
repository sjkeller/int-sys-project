#!/usr/bin/env python3
 
# Import the necessary libraries
from inspect import currentframe
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
 
def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  # Copy read-only buffer to new array
  current_frame = np.array(br.imgmsg_to_cv2(data))

  # Split image in left and right
  width = current_frame.shape[1]
  left = current_frame[:, 0:(width//2)]
  right = current_frame[:, width//2:]

  current_frame[:, width//2, :] = 0
  print("frame size ", current_frame.shape)
  print("left ", left.shape)
  print("right ", right.shape)

  # Display image
  cv2.imshow("left", left)
  cv2.imshow("right", right)
  cv2.imshow("camera", current_frame)

  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
