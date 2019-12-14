#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud 
import numpy as np

class Node:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.prev_transform = 0
        self.transform = 0
        self.point_cloud = 0
        self.pub = rospy.Publisher("/transformed", PointCloud2)
    def pcListener(self, message):
        self.point_cloud = message
        try:
            self.transform = self.tfBuffer.lookup_transform("usb_cam", "royale_camera_optical_frame", rospy.Time())
            # check to see if transform is the same as last time, if it is, delete it
            if(self.transform.transform.translation.x == self.prev_transform):
                self.transform = 0
            # update the previous transform
            self.prev_transform = self.transform.transform.translation.x
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.transform = 0
        if(self.transform != 0):
            out_cloud = do_transform_cloud(self.point_cloud, self.transform)
            print(self.transform)
            self.pub.publish(out_cloud)
        else:
            print("Transform Error")


if __name__ == '__main__':
    rospy.init_node('optimusprime')
    server = Node()
    rospy.Subscriber("/cropped", PointCloud2, server.pcListener)
    rospy.spin()
    