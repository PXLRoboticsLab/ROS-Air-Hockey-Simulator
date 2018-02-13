#!/usr/bin/env python3
import rospy
from src.screen import AirhockeyScreen

if __name__ == '__main__':
    rospy.init_node('airhockey')
    airhockey = AirhockeyScreen().root.mainloop()