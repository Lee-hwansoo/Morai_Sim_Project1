#!/usr/bin/env python3
# -*- coding: utf -8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class ImageToAVI:
    def __init__(self):
        rospy.init_node('img_to_avi', anonymous=True)
        self.image_subscriber = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback)
        self.video_writer = None
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cleanup)

    def callback(self, data):
        try:
            cv_image = self.convert_msg_to_cv2(data)
            self.write_frame(cv_image)
        except Exception as e:
            rospy.logerr(e)

    def convert_msg_to_cv2(self, data):
        return self.bridge.compressed_imgmsg_to_cv2(data)

    def write_frame(self, img):
        try:
            if self.video_writer is None:
                height, width, _ = img.shape
                out_filename = 'output_video.mp4'
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(out_filename, fourcc, 30.0, (width, height))

            self.video_writer.write(img)
            rospy.loginfo("Frame written to video.")
        except Exception as e:
            rospy.logerr(e)

    def cleanup(self):
        if self.video_writer is not None:
            self.video_writer.release()

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber = ImageToAVI()
        image_subscriber.start()
    except rospy.ROSInterruptException:
        pass
