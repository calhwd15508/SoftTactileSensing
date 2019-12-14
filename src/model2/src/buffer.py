#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

class Node:
    def __init__(self):
        self.point_cloud = 0
    def pcListener(self, message):
        self.point_cloud = message

if __name__ == '__main__':
    rospy.init_node('bufferer')
    server = Node()
    rospy.Subscriber("/royale_camera_driver/point_cloud", PointCloud2, server.pcListener)
    pub = rospy.Publisher("/buffer", PointCloud2)
    while not rospy.is_shutdown():
        raw_input("Enter for next measurement: ")
        if(server.point_cloud != 0):
            pub.publish(server.point_cloud)
        else:
        	print("Point Cloud Error")

    