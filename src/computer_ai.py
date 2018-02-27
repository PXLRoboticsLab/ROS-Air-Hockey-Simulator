#!/usr/bin/env python
import cv2
import rospy
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ComputerAI:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/airhockey/simulator/image', Image, self.callback)
        self.previous_center = None
        self.frames_skipped = 0

    def callback(self, data):
        try:
            hockey_field = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            height, width, _ = hockey_field.shape
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(hockey_field, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        rows = gray.shape[0]
        puck = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=50, maxRadius=55)

        pods = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=90, maxRadius=95)

        if puck is not None:
            circles = np.uint16(np.around(puck))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(hockey_field, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(hockey_field, center, radius, (255, 0, 255), 3)

                if self.previous_center is not None:
                    rad = self.calculate_slope(self.previous_center, center)
                    print(rad)
                    print('Rad: {}'.format(rad))

                    c = math.cos(rad)
                    s = math.sin(rad)
                    p1 = (int(center[0] - c * 4096), int(center[1] - s * 4096))
                    p2 = (int(center[0] + c), int(center[1] + s))

                    cv2.line(hockey_field, p1, p2, (0, 0, 255), 3, cv2.LINE_AA)

                self.previous_center = center

        if pods is not None:
            circles = np.uint16(np.around(pods))
            for i in circles[0, :]:
                center = (i[0], i[1])
                if center[0] > 795:
                    # circle center
                    cv2.circle(hockey_field, center, 1, (0, 100, 100), 3)
                    # circle outline
                    radius = i[2]
                    cv2.circle(hockey_field, center, radius, (240, 0, 169), 3)

        cv2.imshow('Segmentation', hockey_field)
        if cv2.waitKey(1) == 27:
            exit(0)


    def calculate_slope(self, point1, point2):
        axis1 = float(point1[1]) - float(point2[1])
        axis2 = float(point1[0]) - float(point2[0])
        return math.atan2(axis1, axis2)


if __name__ == '__main__':
    rospy.init_node('airhockey_ai', anonymous=True)
    ai = ComputerAI()
    rospy.spin()
