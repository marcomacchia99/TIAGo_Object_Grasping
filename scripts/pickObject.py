#!/usr/bin/env python

from operator import sub
import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Pose, Quaternion, Point
from scipy.spatial.transform import Rotation as R
import moveit_commander
from moveit_msgs.msg import Grasp, CollisionObject
from moveit_commander.conversions import pose_to_list
import sys
from std_srvs.srv import Empty, EmptyRequest
from SOFAR_Assignment.srv import ApproachObject, ApproachObjectResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

global sub_target_abs_pose
global robot
global scene
global move_group
global grasp_pose


def goToObject(object_pose):
    global move_group
    global grasp_pose

    object_pose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
    
    # save grasp pose
    grasp_pose = copy.deepcopy(object_pose.pose)
    grasp_pose.position.y-=0.2

    # pre-grasp pose with limits
    object_pose.pose.position.y = -0.2
    object_pose.pose.position.z += 0.1


    rospy.loginfo('attempting to reach:')
    rospy.loginfo([object_pose.pose.position.x,
                   object_pose.pose.position.y, object_pose.pose.position.z])

    move_group.set_pose_target(object_pose.pose)

    move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    response = ApproachObjectResponse()
    response.result = True
    return response


def pick(msg):
    global move_group
    global grasp_pose


    #move to grasp_pose
    rospy.loginfo('move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo('closing gripper...')
    close_gripper()
    rospy.sleep(1)

    # define post_grasp_pose
    post_grasp_pose = copy.deepcopy(grasp_pose)
    post_grasp_pose.position.z += 0.3

    # move to post_grasp_pose
    rospy.loginfo('move to post_grasp_pose...')
    move_group.set_pose_target(post_grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #move to grasp_pose
    rospy.loginfo('move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo('opening gripper...')
    open_gripper()
    rospy.sleep(1)

    rospy.loginfo('Done.')


def close_gripper():

    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

    for i in  range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.0, 0.0]
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)
        rospy.sleep(0.1)  

def open_gripper():

    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

    for i in  range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.044, 0.044]
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)
        rospy.sleep(0.1)

        


# def pickObject(object_pose):
#     global move_group

#     # Setting grasp pose
#     grasp = Grasp()
#     grasp.grasp_pose.header.frame_id = "base_footprint"
#     grasp.grasp_pose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
#     grasp.grasp_pose.pose.position = object_pose.position
#     # grasp.grasp_pose.pose.position.y = max(-0.2, object_pose.pose.position.y-0.3)

#     # Setting pre-grasp approach
#     grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint"
#     grasp.pre_grasp_approach.direction.vector.y = 1.0
#     grasp.pre_grasp_approach.min_distance = 0
#     grasp.pre_grasp_approach.desired_distance = 0

#     # Setting post-grasp retreat
#     grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint"
#     grasp.post_grasp_retreat.direction.vector.z = 1.0
#     grasp.post_grasp_retreat.min_distance = 0.0
#     grasp.post_grasp_retreat.desired_distance = 0.0

#     # Setting posture of eef before grasp
#     openGripper(grasp.pre_grasp_posture)

#     # Setting posture of eef during grasp
#     closedGripper(grasp.grasp_posture)

#     print(grasp)

#     move_group.pick("object", grasp)


# def openGripper(posture):

#     # Add both finger joints of the robot.
#     posture.joint_names.append("gripper_left_finger_joint")
#     posture.joint_names.append("gripper_right_finger_joint")

#     # Set them as open, wide enough for the object to fit.
#     posture.points.append(JointTrajectoryPoint())
#     posture.points[0].positions.append(0.04)
#     posture.points[0].positions.append(0.04)
#     posture.points[0].time_from_start = rospy.Duration(0.5)


# def closedGripper(posture):

#     # Add both finger joints of the robot.
#     posture.joint_names.append("gripper_left_finger_joint")
#     posture.joint_names.append("gripper_right_finger_joint")

#     # Set them as closed. */
#     posture.points.append(JointTrajectoryPoint())
#     posture.points[0].positions.append(0.00)
#     posture.points[0].positions.append(0.00)
#     posture.points[0].time_from_start = rospy.Duration(0.5)


if __name__ == '__main__':

    rospy.init_node('PickObject')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander('arm_torso')

    approach_object_service = rospy.Service(
        "/sofar/approach_object", ApproachObject, goToObject)

    grasp_object_service = rospy.Service(
        "/sofar/pick_object", Empty, pick)

    rospy.loginfo("Service ready.")

    open_gripper()

    # object_pose = Pose()
    # object_pose.position.x = 0.6
    # object_pose.position.y = 0.1
    # object_pose.position.z = 0.9 #mettiamo z+0.1

    # goToObject(object_pose)
    # pick()
    #pickObject(object_pose)

    rospy.spin()
