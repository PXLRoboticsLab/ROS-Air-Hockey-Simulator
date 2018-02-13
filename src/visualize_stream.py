#!/usr/bin/env python
import rospy
import argparse
import cv2
import os
from sensor_msgs.msg import Image
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser


class StreamVisualize:
    def __init__(self, topic, record, fps, save_location):
        self.topic = topic
        self.record = record
        self.fps = fps
        self.save_location = save_location

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(self.topic, Image, self.visualize)

        if record:
            self.created_recording = False

        rospy.spin()

    def visualize(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            height, width, _ = cv_image.shape
        except CvBridgeError as e:
            print(e)

        if self.record:
            if not self.created_recording:
                self.created_recording = True
                self.create_recording(width, height)
            self.writer.write(cv_image)
        cv2.imshow(self.topic, cv_image)
        if cv2.waitKey(1) == 27:
            exit(0)

    def create_recording(self, width, height):
        filename = str(datetime.strftime(datetime.now(), '%Y%m%d-%H%M%S'))
        self.writer = cv2.VideoWriter(os.path.join(self.save_location, '{}.avi'.format(filename)),
                                      cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (width, height))


if __name__ == "__main__":
    rospy.init_node('stream_visualization')

    parser = argparse.ArgumentParser()
    parser.add_argument('topic', type=str,
                        help='The ROS image topic to subscribe to.')
    parser.add_argument('--record', action='store_true',
                        help='Record the image stream as video.')
    parser.add_argument('--fps', type=int,
                        help='FPS of the video.', default=30)
    parser.add_argument('--save_location', type=str,
                        help='Video save location.', default='Videos')

    args = parser.parse_args()

    home = expanduser('~')
    full_save_location = os.path.join(home, args.save_location)

    visualize = StreamVisualize(args.topic, args.record, args.fps, full_save_location)
