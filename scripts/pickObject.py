#!/usr/bin/env python

import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import sys

global sub_target_abs_pose
global robot
global scene
global move_group


def goToObject(object_pose):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    global move_group


    move_group.set_pose_target(object_pose)

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()   


if __name__ == '__main__':

    rospy.init_node('PickObject')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander('arm_torso')

    sub_target_abs_pose = rospy.Subscriber(
        '/sofar/target_pose/absolute', Pose, goToObject)


    rospy.spin()
