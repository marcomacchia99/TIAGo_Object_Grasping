#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
import math
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from SOFAR_Assignment.srv import RelToAbsolute, RelToAbsoluteResponse
from SOFAR_Assignment.srv import ApproachObject, ApproachObjectResponse

global action_client
global pub_head_controller
global sub_target_rel_pose
global head_2_movement
global object_found
global pmgoal
global count
global displacement
global object_rel_pose
global object_abs_pose


def go_to_home_position():
    global action_client
    global pmgoal

    rospy.loginfo("Go into the home position")

    pmgoal = PlayMotionGoal()
    pmgoal.motion_name = 'home'
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


def get_object_relative_pose(msg):
    global object_found
    global sub_target_rel_pose
    global count
    global object_rel_pose

    object_rel_pose = msg
    get_absolute_object_pose()
    
    # count is a counter used to make sure that all the object fits the camera.
    # In order to ensure this, seen the object recognition frequency,
    # the object has to be found for at least 20 times
    count += 1
    if count == 20:
        object_found = True


def prepare_robot():
    global action_client
    global pmgoal

    rospy.loginfo("Go into the initial position")

    pmgoal = PlayMotionGoal()
    pmgoal.motion_name = 'pregrasp'
    pmgoal.skip_planning = False

    action_client.send_goal_and_wait(pmgoal)

    rospy.loginfo("Done.")


def adjust_position():

    global displacement

    rospy.loginfo("Found displacement of %f, moving the robot",displacement)

    pub_vel = rospy.Publisher(
        '/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    velocity = Twist()

    for i in range(21):
        velocity.angular.z = -math.pi/4
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.angular.z = 0
    pub_vel.publish(velocity)
    rospy.sleep(1)

    for i in range(round(displacement/0.25)*10):
        velocity.linear.x = 0.25
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.linear.x = 0
    pub_vel.publish(velocity)
    rospy.sleep(1)

    for i in range(20):
        velocity.angular.z = math.pi/4
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.angular.z = 0
    pub_vel.publish(velocity)
    rospy.sleep(1)

    rospy.loginfo("Done.")


def get_absolute_object_pose():
    global object_abs_pose
    global object_rel_pose

    try:
        rel_to_absolute_pose = rospy.ServiceProxy('/sofar/rel_to_absolute_pose', RelToAbsolute)
        msg = PoseStamped( pose = object_rel_pose)
        msg.header.frame_id = 'xtion_rgb_frame'

        object_abs_pose = rel_to_absolute_pose(msg).absolute_pose.pose

    except rospy.ServiceException as e:
        rospy.logerr("Could not connect to /sofar/rel_to_absolute_pose service")
        exit()

if __name__ == '__main__':

    rospy.init_node('PickClient')

    action_client = SimpleActionClient('/play_motion', PlayMotionAction)

    if not action_client.wait_for_server(rospy.Duration(20)):
        rospy.logerr("Could not connect to /play_motion")
        exit()

    pmgoal = PlayMotionGoal()

    go_to_home_position()

    pub_head_controller = rospy.Publisher(
        '/head_controller/command', JointTrajectory, queue_size=1)

    sub_target_rel_pose = rospy.Subscriber(
        '/sofar/target_pose/relative', Pose, get_object_relative_pose)

    head_2_movement = 0
    object_found = False
    count = 0

    move_head()

    rospy.wait_for_service('/sofar/rel_to_absolute_pose')

    print(object_abs_pose.position.y)
    displacement = -0.05-object_abs_pose.position.y

    if displacement > 0:
        adjust_position()

    prepare_robot()

    rospy.wait_for_service('/sofar/approach_object')
    try:
        sub_target_rel_pose.unregister()

        approach_object = rospy.ServiceProxy('/sofar/approach_object', ApproachObject)
        approach_object(object_abs_pose)

    except rospy.ServiceException as e:
        rospy.logerr("Could not connect to /sofar/rel_to_absolute_pose service")
        exit()

    # dopo pre grasp basta andare avanti di 0.1 lentamente
