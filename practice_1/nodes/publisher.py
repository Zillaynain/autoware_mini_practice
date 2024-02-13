#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('publisher')
rt = rospy.get_param('~rate')
rate = rospy.Rate(rt)
pub = rospy.Publisher('/message', String, queue_size=10)
message = rospy.get_param('~message', 'Hello World!')

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()
