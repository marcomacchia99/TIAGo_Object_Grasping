#!/usr/bin/env python

from operator import sub
import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Pose, Quaternion
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
    global sub_target_abs_pose

    sub_target_abs_pose.unregister()

    # pre-grasp pose with limits
    object_pose.position.y = max(-0.2,object_pose.position.y-0.25)

    rospy.loginfo('attempting to reach:')
    rospy.loginfo([object_pose.position.x,object_pose.position.y,object_pose.position.z])
    object_pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
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

    # object_pose = Pose()
    # object_pose.position.x = 0.5095604789
    # object_pose.position.y = -0.2
    # object_pose.position.z = 0.815161196952
    # goToObject(object_pose)

    rospy.spin()
