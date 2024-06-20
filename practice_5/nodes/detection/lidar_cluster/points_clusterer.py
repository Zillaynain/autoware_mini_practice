#!/usr/bin/env python3

import rospy
import numpy as np

from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN


class PointsCluster:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~cluster_min_size')
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')

        self.cluster_msg_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)
        
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        
        rospy.loginfo("%s - initialized", rospy.get_name())
        
        self.cluster = DBSCAN(eps=self.cluster_epsilon, min_samples=self.min_cluster_size)


    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        
        self.cluster.fit_predict(points)
        labels = self.cluster.labels_

        assert len(points) == len(labels), "Number of points and labels don't match."
        
        filtered_points = points[labels != -1]
        filtered_labels = labels[labels != -1]
        points_labeled = np.column_stack((filtered_points, filtered_labels))
        
        # convert labelled points to PointCloud2 format
        data = unstructured_to_structured(points_labeled, dtype=np.dtype([('x', np.float32),('y', np.float32),('z', np.float32),('label', np.int32)]))
        # publish clustered points message
        cluster_msg = msgify(PointCloud2, data)
        cluster_msg.header.stamp = msg.header.stamp
        cluster_msg.header.frame_id = msg.header.frame_id
        self.cluster_msg_pub.publish(cluster_msg)

        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_cluster', log_level=rospy.INFO)
    node = PointsCluster()
    node.run()
