#!/usr/bin/env python
"""
Path Planning Script for Soft Touch Sensor Facial Recognition
Author: Howard Zhang
Used the script from Lab 7 of C106A from Valmik Prabhu
"""
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

def main():
    """
    Main Script
    """

    # Path Planner - right_arm for sawyer

    planner = PathPlanner("right_arm")

    ##
    ## OBSTACLES
    ##
    # poses = PoseStamped()
    # poses.pose.position.x = .5
    # poses.pose.position.y = 0
    # poses.pose.position.z = 0
    # poses.pose.orientation.x = 0
    # poses.pose.orientation.y = 0
    # poses.pose.orientation.z = 0
    # poses.pose.orientation.w = 1
    # poses.header.frame_id = "base"

    # planner.add_box_obstacle(np.array([.4,1.2,.1]), "Howard", poses)


    # ORIENTATION CONSTRAINT
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    #FULL PATH
    path = [[.8, .05, -.23, "1"],
            [.6, -.3, 0.0, "2"],
            [.6, -.1, .1, "3"]]

    while not rospy.is_shutdown():
        for pos in path:
            while not rospy.is_shutdown():
                try:
                    goal = PoseStamped()
                    goal.header.frame_id = "base"

                    goal.pose.position.x = pos[0]
                    goal.pose.position.y = pos[1]
                    goal.pose.position.z = pos[2]

                    goal.pose.orientation.x = 0.0
                    goal.pose.orientation.y = -1.0
                    goal.pose.orientation.z = 0.0
                    goal.pose.orientation.w = 0.0

                    plan = planner.plan_to_pose(goal, [orien_const])
                    raw_input("Press <Enter> to move the right arm to goal pose " + pos[3] + ":")
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")
                except Exception as e:
                    print e
                    traceback.print_exc()
                else:
                    break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
