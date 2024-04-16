#!/usr/bin/env python3

import rospy
import numpy as np

from shapely import MultiPoint
from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify

from sensor_msgs.msg import PointCloud2
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point32

from sklearn.cluster import DBSCAN


BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)

class PointsCluster:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~cluster_min_size')
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')
        #self.transform_timeout = rospy.get_param('~transform_timeout')

        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer)

        #self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())
        
        self.cluster = DBSCAN()


    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        print('point shape:', points.shape)
        
        self.cluster.set_params(eps=self.cluster_epsilon, min_samples=self.min_cluster_size)
        self.cluster.fit_predict(points)
        labels = self.cluster.labels_
        print('labels shape:', labels.shape)
        assert len(points) == len(labels), "Number of points and labels don't match."


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_cluster', log_level=rospy.INFO)
    node = PointsCluster()
    node.run()
