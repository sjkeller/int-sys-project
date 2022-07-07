#!/usr/bin/env python3
 
# Import the necessary libraries
from pathlib import Path
import cv2  # OpenCV library
import rospy  # Python library for ROS
import numpy as np
from cv_bridge import \
    CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image  # Image is the message type

# get calibration data from xml file
calib_file = cv2.FileStorage()
home = Path.home()
calib_file.open(str(home) + "/isp-2022/jetson_nano/catkin_ws/src/camera/scripts/stereoMap.xml", cv2.FileStorage_READ)

stereoMapL_x = calib_file.getNode('stereoMapL_x').mat()
stereoMapL_y = calib_file.getNode('stereoMapL_y').mat()
stereoMapR_x = calib_file.getNode('stereoMapR_x').mat()
stereoMapR_y = calib_file.getNode('stereoMapR_y').mat()

def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub_corr_both = rospy.Publisher('/camera/frame_corr_both', Image, queue_size=3)
  pub_both = rospy.Publisher('/camera/frame_both', Image, queue_size=3)

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cap = cv2.VideoCapture(cv2.CAP_ANY)

  # Used to convert between ROS and OpenCV images
  bridge = CvBridge()
  # While ROS is still running.
  while not rospy.is_shutdown():

      # Capture frame-by-frame
      ret, frame = cap.read()
      if ret:

        # Split image in left and right
        width = frame.shape[1]
        raw_left = frame[:, 0:(width//2)]
        raw_right = frame[:, width//2:]

        # apply calibration to both video parts
        right = cv2.remap(raw_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        left = cv2.remap(raw_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT,0)

        rospy.loginfo('publishing video frames')

        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message    hzzjoi

        frame_corr = np.hstack((left, right))


        pub_both.publish(bridge.cv2_to_imgmsg(frame, encoding = "rgb8"))
        pub_corr_both.publish(bridge.cv2_to_imgmsg(frame_corr, encoding = "rgb8"))

      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  #cv2.namedWindow('raw',cv2.WINDOW_NORMAL)
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
