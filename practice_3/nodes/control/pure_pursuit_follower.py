#!/usr/bin/env python3

import rospy

from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped

from shapely.geometry import LineString, Point
from shapely import prepare, distance

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_ls = None
        
        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        
    def path_callback(self, msg):
        # convert waypoints to shapely linestring
        path_linestring = LineString([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])
        
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(path_linestring)
        self.path_ls = path_linestring
        
    def current_pose_callback(self, msg):
        
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        vehicle_cmd.ctrl_cmd.steering_angle = 0.2
        vehicle_cmd.ctrl_cmd.linear_velocity = 10.0
        self.vehicle_cmd_pub.publish(vehicle_cmd)
        
        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        d_ego_from_path_start = self.path_ls.project(current_pose)
        print("distance", d_ego_from_path_start)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
