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
        font = cv2.FONT_HERSHEY_SIMPLEX

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
                cv2.putText(hockey_field, 'center {}'.format(center), center, font, 1, (255, 255, 0), 2, cv2.LINE_AA)
                # circle outline
                radius = i[2]
                cv2.circle(hockey_field, center, radius, (255, 0, 255), 3)

                if self.previous_center is not None:
                    rad = self.calculate_slope(self.previous_center, center)
                    print(rad)
                    cv2.putText(hockey_field, 'angle {}'.format(rad), (center[0], center[1] - 30) , font, 1, (255, 255, 0), 2,cv2.LINE_AA)
                    c = math.cos(rad)
                    s = math.sin(rad)
                    p1 = (int(center[0] - c * 4096), int(center[1] - s * 4096))
                    p2 = (int(center[0] + c), int(center[1] + s))

                    point = self.calculate_intersect(center, rad, s, c)
                    point2 = self.calculate_intersect2(center, rad, s, c)

                    cv2.circle(hockey_field, point, 10, (255, 0, 255), 3)
                    cv2.circle(hockey_field, point2, 10, (255, 0, 255), 3)
                    cv2.putText(hockey_field,'Point {}'.format(point),point, font, 1,(0,255,0),2,cv2.LINE_AA)
                    cv2.putText(hockey_field,'Point2 {}'.format(point2),point2, font, 1,(0,255,0),2,cv2.LINE_AA)
                    cv2.line(hockey_field, p1, p2, (0, 0, 255), 3, cv2.LINE_AA)

                self.previous_center = center

        if pods is not None:
            circles = np.uint16(np.around(pods))
            for i in circles[0, :]:
                center = (i[0], i[1])
                if center[0] > 795:
                    # circle center
                    cv2.circle(hockey_field, center, 1, (0, 100, 100), 3)
                    cv2.putText(hockey_field, 'center {}'.format(center), center, font, 1, (0, 255, 0), 2,cv2.LINE_AA)
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

    def calculate_intersect(self, pos, rad, s, c):
        if rad > 0:
            xdiff = c * float(pos[1])
            print("Xdiff: {}".format(xdiff))
            print("Point: {}".format((int(pos[0] - xdiff), 0)))
            return int(pos[0] - xdiff), 0
        else:
            xdiff = c * (1000.0-float(pos[1]))
            print("Xdiff: {}".format(xdiff))
            print("Point: {}".format((int(pos[0] - xdiff), 1000)))
            return int(pos[0] - xdiff), 1000

    def calculate_intersect2(self, pos, rad, s, c):
        if math.degrees(rad) > 90 or math.degrees(rad) < -90:
            ydiff = s * (1550-float(pos[0]))
            print("Ydiff: {}".format(ydiff))
            print("Point2: {}".format((int(pos[0] - ydiff), 0)))
            return 1550, int(pos[1] - ydiff)
        else:
            ydiff = s * float(pos[0])
            print("Ydiff: {}".format(ydiff))
            print("Point2: {}".format((int(pos[0] - ydiff), 1000)))
            return 0, int(pos[1] - ydiff)


if __name__ == '__main__':
    rospy.init_node('airhockey_ai', anonymous=True)
    ai = ComputerAI()
    rospy.spin()
