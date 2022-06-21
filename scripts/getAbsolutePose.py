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

global sub_targetPose


def computeAbsPose(rel_pose):

    global seq
    global tfBuffer
    global listener
    global abs_pose

    trans_base_camera = tfBuffer.lookup_transform(
        'xtion_optical_frame', 'base_footprint', rospy.Time())

    point_from_camera = PointStamped()
    point_from_camera.header.frame_id = '/xtion_optical_frame'
    point_from_camera.header.seq = seq
    seq = seq + 1
    point_from_camera.header.stamp = rospy.Time()

    point_from_camera.point.x = rel_pose.position.x
    point_from_camera.point.y = rel_pose.position.y
    point_from_camera.point.z = rel_pose.position.z

    abs_pose.position = do_transform_point(
        point_from_camera, trans_base_camera).point

    q0 = [trans_base_camera.transform.rotation.x, trans_base_camera.transform.rotation.y,
          trans_base_camera.transform.rotation.z, trans_base_camera.transform.rotation.w]
    q1 = [rel_pose.orientation.x, rel_pose.orientation.y,
          rel_pose.orientation.z, rel_pose.orientation.w]
    q = quaternion_multiply(q0, q1)

    abs_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    print(abs_pose)


if __name__ == '__main__':
    seq = 1

    rospy.init_node('GetAbsolutePose')

    sub_targetPose = rospy.Subscriber(
        '/sofar/target_rel_pose', Pose, computeAbsPose)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    abs_pose = Pose()

    rospy.spin()
