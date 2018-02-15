#!/usr/bin/env python
import os
import rospy
import pymunk
from PIL import Image
from geometry_msgs.msg import Twist


class Pod:
    dir = os.path.dirname(__file__)
    _vel_x = 0
    _vel_y = 0

    def __init__(self, x, y, radius, mass, player, topic):
        self.player = player
        self.image = Image.open(os.path.join(self.dir, '../res/player_pod.png'))
        self.width, self.height = self.image.size

        self.inertia = pymunk.moment_for_circle(mass, 0, radius)
        self.body = pymunk.Body(mass, self.inertia, pymunk.Body.KINEMATIC)
        self.body.position = x, y
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.elasticity = 0.95

        self.velocity_sub = rospy.Subscriber(topic, Twist, self.ros_velocity)

    def set_velocity(self, vel_x, vel_y):
        self._vel_x = vel_x
        self._vel_y = vel_y

    def ros_velocity(self, velocity):
        self._vel_x = velocity.linear.x
        self._vel_y = velocity.linear.y

    def move(self):
        if not self.body.position.x - 95 <= 25:
            self.body.position = self.body.position.x + self._vel_x, self.body.position.y + self._vel_y
        else:
            self.body.position = 120, self.body.position.y
        if not self.body.position.x + 95 >= 1570:
            self.body.position = self.body.position.x + self._vel_x, self.body.position.y + self._vel_y
        else:
            self.body.position = 1475, self.body.position.y

        if not self.body.position.y - 95 <= 25:
            self.body.position = self.body.position.x + self._vel_x, self.body.position.y + self._vel_y
        else:
            self.body.position = self.body.position.x, 120
        if not self.body.position.y + 95 >= 975:
            self.body.position = self.body.position.x + self._vel_x, self.body.position.y + self._vel_y
        else:
            self.body.position = self.body.position.x, 880
        return self.get_position()

    def set_position(self, x, y):
        self.body.position = x, y
        return self.get_position()

    def get_position(self):
        x_pos = int(self.body.position.x - self.width / 2)
        y_pos = int(self.body.position.y - self.height / 2)
        return x_pos, y_pos

    def key_pressed(self, event):
        if 'Right' == event.keysym or event.keysym  == 'd':
            self._vel_x = 15
        elif 'Left' == event.keysym or event.keysym == 'q':
            self._vel_x = -15
        elif 'Up' == event.keysym or event.keysym == 'z':
            self._vel_y = -15
        elif 'Down' == event.keysym or event.keysym == 's':
            self._vel_y = 15

    def key_released(self, event):
        if 'Right' == event.keysym or event.keysym == 'd' or 'Left' == event.keysym or event.keysym == 'q':
            self._vel_x = 0
        elif 'Up' == event.keysym or event.keysym == 'z' or 'Down' == event.keysym or event.keysym == 's':
            self._vel_y = 0
