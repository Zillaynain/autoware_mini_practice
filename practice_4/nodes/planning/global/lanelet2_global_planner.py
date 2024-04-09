#!/usr/bin/env python3


import rospy
import time

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Lane, Waypoint, WaypointState



class Lanelet2GlobalPlanner:

    def __init__(self):
    
        # Parameters
        
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        self.output_frame = rospy.get_param("~output_frame")
        self.distance_to_goal_limit = rospy.get_param("~distance_to_goal_limit")
        
        self.goal_point = None
        self.current_location = None
        self.waypoints = []


        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_name, projector)
        
        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,lanelet2.traffic_rules.Participants.VehicleTaxi)

        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Lane, queue_size=10, latch=True)
    
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback) 
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback) 
        
        
    def goal_callback(self, msg):
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)
        
        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        #get start and end lanelets
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
        
        # find routing graph
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)
        
        if route is None:
            return 
        else:
            # find shortest path
            path = route.shortestPath()
            # this returns LaneletSequence to a point where lane change would be necessary to continue
            path_no_lane_change = path.getRemainingLane(start_lanelet)
            self.waypoints = self.lanelet_sequence_to_waypoints(path_no_lane_change)
            print("the",self.waypoints[-1].pose.pose.position.x)
            self.publish_waypoints(self.waypoints)
            

    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is not None:
            dist = ((self.goal_point.x - self.current_location.x)**2 + (self.goal_point.y - self.current_location.y)**2)**0.5

            if dist < self.distance_to_goal_limit:
                self.waypoints = []
                #self.goal_point = None
                self.publish_waypoints(self.waypoints)
                rospy.logwarn("the goal has been reached, path has been cleared. ")
                return
            

    def lanelet_sequence_to_waypoints(self,path_no_lane_change): 
        waypoints = []
        for lanelet in path_no_lane_change:
                if 'speed_ref' in lanelet.attributes:
                    speed_kmh = float(lanelet.attributes['speed_ref'])
                    speed = speed_kmh / 3.6
                else:
                    speed_kmh = rospy.get_param('~speed_limit') 
                    speed = speed_kmh / 3.6
                for point in lanelet.centerline:
                    # Check if it's not the first waypoint or if the previous waypoint is not the same as the current point
                    if not waypoints or (waypoints[-1].pose.pose.position.x != point.x and waypoints[-1].pose.pose.position.y != point.y):
                        # create Waypoint and get the coordinats from lanelet.centerline points
               
                        waypoint = Waypoint()
                        waypoint.pose.pose.position.x = point.x
                        waypoint.pose.pose.position.y = point.y
                        waypoint.pose.pose.position.z = point.z
                        waypoint.twist.twist.linear.x = speed
                        waypoints.append(waypoint)
        self.goal_point.x = waypoints[-1].pose.pose.position.x
        self.goal_point.y = waypoints[-1].pose.pose.position.y
        return waypoints

    def publish_waypoints(self, waypoints):
        lane = Lane()      
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.waypoints_pub.publish(lane)
           
    def run(self):
        rospy.spin()
     

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()
