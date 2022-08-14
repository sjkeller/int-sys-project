#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Mediator:
    def __init__(self, cfg):
        self.yolo_pub = rospy.Publisher(
            '/camera/rgb/image_raw', Image, queue_size=1
        )
        self.cfg = cfg
        self.br = CvBridge()
        for n in ['frame_slice', 'scale_factor', 'calib']:
            rospy.loginfo(f'{n}: {self.cfg[n]}')

    def mediate(self, data):
        frame = self.br.imgmsg_to_cv2(data)
        img, encoding = self.transform_frame(frame)
        rospy.loginfo(
            f'Mediating video frame {frame.shape} to yolo: {img.shape}'
        )
        self.yolo_pub.publish(self.br.cv2_to_imgmsg(img, encoding=encoding))

    def transform_frame(self, frame):
        width = frame.shape[1]
        center = width//2
        fslice = frame[:, center:] if self.cfg['frame_slice'] == 'right' else frame[:, :center]
        fslice = cv2.cvtColor(fslice, cv2.COLOR_BGR2RGB)
        scale_factor = self.cfg['scale_factor']
        interpolation = cv2.INTER_AREA if scale_factor < 1 else cv2.INTER_LINEAR
        if scale_factor != 1:
            fslice = cv2.resize(
                fslice, None, fx=scale_factor,
                fy=scale_factor, interpolation=interpolation
            )
        return fslice, "rgb8"


def run():
    rospy.init_node('yolo_mediator_py', log_level=rospy.INFO)

    mediator = Mediator({
        k: rospy.get_param(k)
        for k in ['frame_slice', 'scale_factor', 'calib']
    })

    frame_both_topic = '/camera/frame_corr_both' if rospy.get_param(
        'calib') else '/camera/frame_both'
    rospy.Subscriber(frame_both_topic, Image, mediator.mediate)
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
