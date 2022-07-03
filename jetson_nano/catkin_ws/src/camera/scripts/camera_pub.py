#!/usr/bin/env python3
 
# Import the necessary libraries
import cv2  # OpenCV library
import rospy  # Python library for ROS
from cv_bridge import \
    CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image  # Image is the message type


def publish_message(frame_slice='right'):
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('video_frames', Image, queue_size=3)
  pub_slice = rospy.Publisher('camera/rgb/image_raw', Image, queue_size=2)

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
     
      #print("contrast", cap.get(cv2.CAP_PROP_CONTRAST))
      #print("saturation", cap.get(cv2.CAP_PROP_SATURATION))
      #print("brightness", cap.get(cv2.CAP_PROP_BRIGHTNESS))

      # Capture frame-by-frame
      ret, frame = cap.read()
      if ret:
        width = frame.shape[1]
        center = width//2
        fslice = frame[:, center:] if frame_slice == 'right' else frame[:, :center]
        fslice = cv2.cvtColor(fslice, cv2.COLOR_BGR2RGB)

        rospy.loginfo('publishing video frame')

        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(bridge.cv2_to_imgmsg(frame, encoding="rgb8"))
        pub_slice.publish(bridge.cv2_to_imgmsg(fslice, encoding = "rgb8"))

      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  #cv2.namedWindow('raw',cv2.WINDOW_NORMAL)
  import sys
  frame_slice = sys.argv[1] if len(sys.argv) > 1 else 'right'
  try:
    publish_message(frame_slice)
  except rospy.ROSInterruptException:
    pass
