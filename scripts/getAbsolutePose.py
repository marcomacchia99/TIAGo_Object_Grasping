#!/usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import *
import ros_numpy
from geometry_msgs.msg import Pose, Quaternion, PointStamped
from scipy.spatial.transform import Rotation as R

global seq
global tfBuffer
global listener
global abs_pose

global sub_target_rel_pose
global pub_target_abs_pose
global pub_target_abs_pose_stamped



def computeAbsPose(rel_pose):

    global seq
    global tfBuffer
    global listener
    global abs_pose
    global pub_target_abs_pose
    global pub_target_abs_pose_stamped


    trans_base_camera = tfBuffer.lookup_transform('base_footprint', 
        'xtion_rgb_frame', rospy.Time())
    

    point_from_camera = PointStamped(point=rel_pose.position)

    abs_pose.position = do_transform_point(
        point_from_camera, trans_base_camera).point

    q0 = [trans_base_camera.transform.rotation.x, trans_base_camera.transform.rotation.y,
          trans_base_camera.transform.rotation.z, trans_base_camera.transform.rotation.w]
    q1 = [rel_pose.orientation.x, rel_pose.orientation.y,
          rel_pose.orientation.z, rel_pose.orientation.w]
    q = quaternion_multiply(q1, q0)

    abs_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    pub_target_abs_pose.publish(abs_pose)
    pose = PoseStamped(pose = abs_pose)
    pose.header.frame_id='base_footprint'
    pub_target_abs_pose_stamped.publish(pose)


if __name__ == '__main__':
    seq = 1

    rospy.init_node('GetAbsolutePose')

    sub_target_rel_pose = rospy.Subscriber(
        '/sofar/target_pose/relative', Pose, computeAbsPose)

    pub_target_abs_pose = rospy.Publisher(
        '/sofar/target_pose/absolute', Pose, queue_size=1)
    pub_target_abs_pose_stamped = rospy.Publisher(
        '/sofar/target_pose/absolute/stamped', PoseStamped, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    abs_pose = Pose()

    rospy.spin()
