#!/usr/bin/env python
import rospy
from screen import AirhockeyScreen

if __name__ == '__main__':
    rospy.init_node('airhockey')
    airhockey = AirhockeyScreen().root.mainloop()