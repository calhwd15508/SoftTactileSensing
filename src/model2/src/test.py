#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def pcListener(message):
    print(len(message.data))

if __name__ == '__main__':
    rospy.init_node('test')
    rospy.Subscriber("/out_cloud", PointCloud2, pcListener)
    rospy.spin()

    