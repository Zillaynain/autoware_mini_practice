#!/usr/bin/env python3


import rospy
import time

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from geometry_msgs.msg import PoseStamped


class Lanelet2MapVisualizer:

    def __init__(self):
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
    def goal_callbCK()

