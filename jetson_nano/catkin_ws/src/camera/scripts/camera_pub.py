#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
  pub_left = rospy.Publisher('video_left', Image, queue_size=10)
  pub_right = rospy.Publisher('video_right', Image, queue_size=10)  

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  #cap = cv2.VideoCapture(cv2.CAP_ANY)
  cap = cv2.VideoCapture(0)

  # Used to convert between ROS and OpenCV images
  bridge = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
      width = frame.shape[1]
      left = frame[:, 0:(width//2)]
      right = frame[:, width//2:]
      #cv2.imshow("pub show", frame)
      #cv2.waitKey(1)
      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        pub_left.publish(bridge.cv2_to_imgmsg(left, encoding="bgr8"))
        pub_right.publish(bridge.cv2_to_imgmsg(right, encoding="bgr8"))

      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
