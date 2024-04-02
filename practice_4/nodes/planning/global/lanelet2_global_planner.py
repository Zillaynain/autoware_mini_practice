#!/usr/bin/env python3


import rospy
import time

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from geometry_msgs.msg import PoseStamped


class Lanelet2GlobalPlanner:

    def __init__(self):
    
        # Parameters
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_name, projector)
    
    
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
    def goal_callback(self, msg):
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,msg.pose.orientation.w, msg.header.frame_id)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()
