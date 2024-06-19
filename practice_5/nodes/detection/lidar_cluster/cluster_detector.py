#!/usr/bin/env python3

import rospy
import numpy as np

from shapely import MultiPoint
from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify, msgify

from sensor_msgs.msg import PointCloud2
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point32


BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)

class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~min_cluster_size')
        self.output_frame = rospy.get_param('/detection/output_frame')
        self.transform_timeout = rospy.get_param('~transform_timeout')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        

        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_clustered', PointCloud2, self.cluster_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())


    def cluster_callback(self, msg):

        data = numpify(msg)
        if data['label'] is None:
            self.objects_pub.publish([], msg.header.stamp, self.output_frame)
        else:
            label = data['label']
            
        points = structured_to_unstructured(data, dtype=np.float32)
        
        if msg.header.frame_id != self.output_frame:
            try:
                transform = self.tf_buffer.lookup_transform(self.output_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(self.transform_timeout))
            except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
               rospy.logwarn("%s - %s", rospy.get_name(), e)
               return
            tf_matrix = numpify(transform.transform).astype(np.float32)
            points = points.copy()
            points[:,3] = 1
            points = points.dot(tf_matrix.T)

        header = Header(**{
        'stamp': msg.header.stamp,
        'frame_id': self.output_frame
        })
        objects=DetectedObjectArray(header=header)

        if len(label) == 0:
            clusters = 0
        else:
            clusters = np.max(label) + 1
            
        # create mask
        for i in range(clusters):
            mask = (label == i)
            points3d = points[mask,:3]
            if len(points3d) < self.min_cluster_size:
                continue
            centroid_x, centroid_y, centroid_z = np.mean(points3d, axis=0)
            # create convex hull
            points_2d = MultiPoint(points[mask,:2])
            hull = points_2d.convex_hull
            convex_hull_points = [Point32(x, y, centroid_z) for x, y in hull.exterior.coords]
            
            self.Object = DetectedObject(header=header)
            self.Object.convex_hull.polygon.points = convex_hull_points
            self.Object.pose.position.x = centroid_x
            self.Object.pose.position.y = centroid_y
            self.Object.pose.position.z = centroid_z
            self.Object.id = i
            self.Object.label = "unknown"
            self.Object.color = BLUE80P
            self.Object.valid = True
            self.Object.space_frame = self.output_frame
            self.Object.pose_reliable = True
            self.Object.velocity_reliable = False
            self.Object.acceleration_reliable = False
            objects.objects.append(self.Object)
        print(objects)
        self.objects_pub.publish(objects)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cluster_detector', log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()
