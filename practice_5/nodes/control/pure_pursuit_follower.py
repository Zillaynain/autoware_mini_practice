#!/usr/bin/env python3

import rospy
import math
import numpy as np
from tf.transformations import euler_from_quaternion

from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped

from shapely.geometry import LineString, Point
from shapely import prepare, distance

from scipy.interpolate import interp1d

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_linestring = None
        self.distance_to_velocity_interpolator = None
        
        # Reading in the parameter values
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/wheel_base")
        
        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        
    def path_callback(self, msg):
        if len(msg.waypoints) !=0:
            # convert waypoints to shapely linestring
            path_line= LineString([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])
            # prepare path - creates spatial tree, making the spatial queries more efficient
            prepare(path_line)
            path_linestring = path_line
        
        
            # Create a distance to velocity interpolator for the path
            # collect waypoint x and y coordinates
            waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])

            # Calculate distances between points
            distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
        
            # add 0 distance in the beginning
            distances = np.insert(distances, 0, 0)
        
            # Extract velocity values at waypoints
            velocities = np.array([w.twist.twist.linear.x for w in msg.waypoints])
            distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear')
        
            self.path_linestring = path_linestring
            #print(self.path_linestring)
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator
        else:
            self.path_linestring = None
    
    
    def current_pose_callback(self, msg):
      
        if self.path_linestring is not None:
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            d_ego_from_path_start = self.path_linestring.project(current_pose)
            # using euler_from_quaternion to get the heading angle
            _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
            lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start+self.lookahead_distance)
            lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)
            lookahead_distance = current_pose.distance(lookahead_point)
            steering_angle = np.arctan(2*self.wheel_base*np.sin(lookahead_heading-heading)/lookahead_distance)
            velocity = self.distance_to_velocity_interpolator(d_ego_from_path_start)
        else:
            steering_angle = 0.0
            velocity = 0.0
            
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = velocity
        self.vehicle_cmd_pub.publish(vehicle_cmd)
    
    def acurrent_pose_callback(self, msg):
    
        # Check if self.path_linestring is not None   
        if msg.pose is not None:
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            d_ego_from_path_start = self.path_linestring.project(current_pose)
            # using euler_from_quaternion to get the heading angle
            _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
            lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start+self.lookahead_distance)
            lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)
            lookahead_distance = current_pose.distance(lookahead_point)
            steering_angle = np.arctan(2*self.wheel_base*np.sin(lookahead_heading-heading)/lookahead_distance)
            velocity = self.distance_to_velocity_interpolator(d_ego_from_path_start)
        else:
            steering_angle = 0.0
            velocity = 0.0
            
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = velocity
        self.vehicle_cmd_pub.publish(vehicle_cmd)

        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
