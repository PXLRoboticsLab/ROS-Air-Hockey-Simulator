#!/usr/bin/env python
import os
import pymunk
import random
from PIL import Image


def constant_velocity(body, gravity, damping, dt):
    body.velocity = body.velocity.normalized() * 2500


class Puck:
    dir = os.path.dirname(__file__)

    def __init__(self, x, y, radius, mass):
        self.image = Image.open(os.path.join(self.dir, '../res/puck.png'))
        self.width, self.height = self.image.size

        self.inertia = pymunk.moment_for_circle(mass, 0, radius)
        self.body = pymunk.Body(mass, self.inertia)
        self.body.apply_impulse_at_local_point(pymunk.Vec2d(random.choice([-1, 1]), random.choice([-1, 1])))
        self.body.velocity_func = constant_velocity
        self.body.position = x, y
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.elasticity = 0.95

    def get_position(self):
        x_pos = int(self.body.position.x - self.width / 2)
        y_pos = int(self.body.position.y - self.height / 2)
        return x_pos, y_pos

    def check_goal(self, score):
        if 340 <= self.body.position.y <= 660:
            if (self.body.position.x - 100) <= 0:
                score['player_2'] += 1
                self.body.position = 795, 500
            elif (self.body.position.x + 110) >= 1595:
                self.body.position = 795, 500
                score['player_1'] += 1
