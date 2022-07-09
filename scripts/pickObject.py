#!/usr/bin/env python
"""
.. module:: pickClient
:platform: Linux
:synopsis: Python module to pick up & put down object 
:moduleauthor: M.Macchia S.Pedrazzi M.Haji Hosseini

Publishes to:
    '/gripper_controller/command'
Service :
    "/sofar/approach_object" 
    "/sofar/pick_object"

:Pick Object node description:
    1. TIAGo's manipulator goes to the object
    2. Closes the grippers
    3. TIAGo lifts the object from the table and puts it back on the table
    4. Opens gripper
   
"""

# Header
import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Quaternion
import moveit_commander
import sys
from std_srvs.srv import Empty, EmptyResponse
from SOFAR_Assignment.srv import ApproachObject, ApproachObjectResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# subscriber to target absolute pose
global sub_target_abs_pose

# robot model
global robot

# robot move groups
global move_group

# grasp pose of the EE
global grasp_pose


def goToObject(object_pose):
    """
        Puts the manipulator in the right position to take the object
    Args:
        object_pose (msg): get object pose

    Returns:
        response: operation success status
    """
    # get global variable
    global move_group
    global grasp_pose

    # define final orientation of the EE
    object_pose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

    # set vertical pose of the EE taking care of its shape
    object_pose.pose.position.z += rospy.get_param('z_axis_offset')

    # save grasp pose
    grasp_pose = copy.deepcopy(object_pose.pose)

    # define distance between EE & object
    grasp_pose.position.y -= rospy.get_param('y_axis_offset')

    # pre-grasp pose
    object_pose.pose.position.y = -0.2

    # generate robot status
    rospy.loginfo('PickObject - attempting to reach pre grasp pose')

    # call joint group in order to reach object
    move_group.set_pose_target(object_pose.pose)

    # continue to move the manipulator group to reach the desired position
    move_group.go(wait=True)

    # stop manipulator group after reaching position
    move_group.stop()

    move_group.clear_pose_targets()

    response = ApproachObjectResponse()
    response.result = True
    return response


def pick(msg):
    """
        series of movements to lift and put object back on the table; 
        using move_group of TIAGo's manipulator
    Args:
        msg (Empty) 
    """
    # get global variable
    global move_group
    global grasp_pose

    # move to grasp_pose
    rospy.loginfo('PickObject - move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #close gripper
    rospy.loginfo('PickObject - closing gripper...')
    close_gripper()
    rospy.sleep(1)

    # define post_grasp_pose
    post_grasp_pose = copy.deepcopy(grasp_pose)
    post_grasp_pose.position.z += rospy.get_param('grasp_height')

    # move to post_grasp_pose
    rospy.loginfo('PickObject - move to post_grasp_pose...')
    move_group.set_pose_target(post_grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # move to grasp_pose
    rospy.loginfo('PickObject - move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #open gripper
    rospy.loginfo('PickObject - opening gripper...')
    open_gripper()

    # move away from cup
    rospy.loginfo('PickObject - move away from cup...')
    grasp_pose.position.y = -0.2
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(1)

    rospy.loginfo('PickObject - Done.')

    response  = EmptyResponse()
    return response


def close_gripper():
    """
        function that closes the gripper of TIAGo's EE
    """

    # publish gripper status on joint trajectory when TIAGo close gripper
    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

    # loop continues until the grippers is closed
    for i in range(10):
        trajectory = JointTrajectory()

        # call joint group for take object
        trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()

        # define the gripper joints configuration
        trajectory_points.positions = [0.0, 0.0]

        # define time duration
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)

        # interval to start next movement
        rospy.sleep(0.1)


def open_gripper():
    """
        function that opens the gripper of TIAGo's EE
    """
    # publish gripper status on joint trajectory when TIAGo open gripper
    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

   # loop continues until the grippers is opened
    for i in range(10):
        trajectory = JointTrajectory()

        # call joint group for take object
        trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()

        # define the gripper joints configuration
        trajectory_points.positions = [0.044, 0.044]

        # define time duration
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)
        
        # interval to start next movement
        rospy.sleep(0.1)


if __name__ == '__main__':

    # ros node initialization --> Pick Object
    rospy.init_node('PickObject')

    # initialize moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    # define group of joint to adjust the height of TIAGo
    move_group = moveit_commander.MoveGroupCommander('arm_torso')

    # service pass info to geometry_msgs/Pose pose
    approach_object_service = rospy.Service(
        "/sofar/approach_object", ApproachObject, goToObject)
        
    # service pass info to geometry_msgs/Pose pose
    grasp_object_service = rospy.Service(
        "/sofar/pick_object", Empty, pick)

    rospy.loginfo("PickObject - Services ready.")

    # call function to open gripper
    open_gripper()

    # start infinite loop until it receives a shutdown signal
    rospy.spin()
