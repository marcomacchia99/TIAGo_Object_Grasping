#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

global action_client
global pub_head_controller
global sub_target_rel_pose
global head_2_movement
global object_found
global count


def prepare_robot():
    global action_client

    rospy.loginfo("Go into the initial position")

    pmgoal = PlayMotionGoal()
    pmgoal.motion_name = 'pregrasp'
    pmgoal.skip_planning = False

    action_client.send_goal_and_wait(pmgoal)

    rospy.loginfo("Done.")

def move_head():
    global pub_head_controller
    global head_2_movement
    global object_found

    rospy.loginfo("Moving head")

    while not object_found:

        trajectory = JointTrajectory()
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.0, head_2_movement]
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_head_controller.publish(trajectory)

        rospy.sleep(0.8)
        head_2_movement = max(-1, head_2_movement-0.1)

    rospy.loginfo("Done.")


def stop_head_motion(msg):
    global object_found
    global sub_target_rel_pose
    global count

    #count is used to make sure that all the object fits the camera.
    #In order to ensure this, seen the object recognition frequency,
    #the object has to be found for at least 20 times
    count += 1
    if count == 20:
        object_found = True
        sub_target_rel_pose.unregister()


if __name__ == '__main__':

    rospy.init_node('InitPosition')

    action_client = SimpleActionClient('/play_motion', PlayMotionAction)

    if not action_client.wait_for_server(rospy.Duration(20)):
        rospy.logerr("Could not connect to /play_motion AS")
        exit()

    prepare_robot()

    pub_head_controller = rospy.Publisher(
        '/head_controller/command', JointTrajectory, queue_size=1)

    sub_target_rel_pose = rospy.Subscriber(
        '/sofar/target_pose/relative', Pose, stop_head_motion)

    head_2_movement = 0
    object_found = False
    count = 0

    move_head()
