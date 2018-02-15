#!/usr/bin/env python
import Tkinter as tk
import os
import rospy
import pymunk
import cv2
import numpy as np
from pod import Pod
from puck import Puck
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge


class AirhockeyScreen:
    dir = os.path.dirname(__file__)
    FPS = 60.0

    player1_keys = ['d', 'q', 'z', 's']
    player2_keys = ['Right', 'Left', 'Up', 'Down']

    score = {'player_1': 0, 'player_2': 0}

    def __init__(self):
        self.pub_image = rospy.Publisher('/airhockey/simulator/image', ImageMsg, queue_size=60)
        self.bridge = CvBridge()

        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)

        self.root = tk.Tk()
        self.root.title('Airhockey')
        self.root.resizable(width=False, height=False)
        self.root.geometry("{}x{}+{}+{}".format(1595, 1000, 0, 0))
        self.root.bind("<Escape>", lambda e: self.root.quit())
        #self.root.bind('<Motion>', self.mouse_position)

        self.field_image = Image.open(os.path.join(self.dir, '../res', 'field.png'))

        self.static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.static_line = pymunk.Segment(self.static_body, (0, 40), (1595, 40), 0.0)
        self.static_line.elasticity = 1
        self.space.add(self.static_body, self.static_line)

        self.static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.static_line = pymunk.Segment(self.static_body, (0, 960), (1595, 960), 0.0)
        self.static_line.elasticity = 1
        self.space.add(self.static_body, self.static_line)

        self.static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.static_line = pymunk.Segment(self.static_body, (40, 0), (40, 1000), 0.0)
        self.static_line.elasticity = 1
        self.space.add(self.static_body, self.static_line)

        self.static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.static_line = pymunk.Segment(self.static_body, (1550, 0), (1550, 1000), 0.0)
        self.static_line.elasticity = 1
        self.space.add(self.static_body, self.static_line)

        self.player1_object = Pod(200, 500, 100, 3, 'player1', 'airhockey/player1')
        self.player2_object = Pod(1390, 500, 100, 3, 'player2', 'airhockey/player2')
        self.puck = Puck(795, 500, 60, 1)

        self.space.add(self.player1_object.body, self.player1_object.shape)
        self.space.add(self.player2_object.body, self.player2_object.shape)
        self.space.add(self.puck.body, self.puck.shape)

        self.field = ImageTk.PhotoImage(self.get_field_image(self.player1_object, self.player2_object, self.puck))

        self.canvas = tk.Canvas(self.root, width=1590, height=1000)
        self.canvas.pack()

        self.root.bind_all('<KeyPress>', self.key_pressed)
        self.root.bind_all('<KeyRelease>', self.key_released)

        self.root.after(int(round(1000 / self.FPS, 0)), func=self.tick)

    def tick(self):
        self.player1_object.move()
        self.player2_object.move()
        self.space.step(1 / self.FPS)
        self.render()
        self.puck.check_goal(self.score)
        self.root.after(int(round(1000 / self.FPS, 0)), func=self.tick)

    def render(self):
        self.canvas.delete('all')
        image = self.get_field_image(self.player1_object, self.player2_object, self.puck)
        field = ImageTk.PhotoImage(image)
        self.canvas.create_image((795, 500), image=field)
        self.canvas.image = field
        self.canvas.create_text(799, 75, fill='darkblue', font='Times 80 bold',
                                text='{}:{}'.format(self.score['player_1'], self.score['player_2']))
        """
        Subscribe to this topic to analyze the board in ROS with OpenCV
        To move the player publish a message of geometry_msgs.msg.Twist to /airhockey/player1 or /airhockey/player2.
        """
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR), 'bgr8'))

    def get_field_image(self, *args):
        img_to_show = Image.new('RGB', (1590, 1000), color=(255, 255, 255, 1))
        img_to_show.paste(self.field_image, (0, 0), self.field_image)
        for game_object in args:
            img_to_show.paste(game_object.image, (game_object.get_position()), game_object.image)
        return img_to_show

    def mouse_position(self, event):
        pos_x, pos_y = event.x, event.y
        self.player1_object.set_position(pos_x, pos_y)

    def key_pressed(self, event):
        if event.keysym in self.player1_keys:
            self.player1_object.key_pressed(event)
        elif event.keysym in self.player2_keys:
            self.player2_object.key_pressed(event)

    def key_released(self, event):
        if event.keysym in self.player1_keys:
            self.player1_object.key_released(event)
        elif event.keysym in self.player2_keys:
            self.player2_object.key_released(event)
