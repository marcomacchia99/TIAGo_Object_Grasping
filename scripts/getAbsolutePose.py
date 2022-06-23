#!/usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import *
import ros_numpy
from geometry_msgs.msg import Pose, Quaternion, PointStamped
from SOFAR_Assignment.srv import RelToAbsolute, RelToAbsoluteResponse

global tfBuffer
global listener


def compute_absolute_pose(rel_pose):

    global tfBuffer
    global listener

    rospy.loginfo('New request received')

    trans_base_rel = tfBuffer.lookup_transform('base_footprint',
                                                  rel_pose.relative_pose.header.frame_id, rospy.Time())

    point_from_rel = PointStamped(point=rel_pose.relative_pose.pose.position)

    abs_pose = Pose()

    abs_pose.position = do_transform_point(
        point_from_rel, trans_base_rel).point

    q0 = [trans_base_rel.transform.rotation.x, trans_base_rel.transform.rotation.y,
          trans_base_rel.transform.rotation.z, trans_base_rel.transform.rotation.w]
    q1 = [rel_pose.relative_pose.pose.orientation.x, rel_pose.relative_pose.pose.orientation.y,
          rel_pose.relative_pose.pose.orientation.z, rel_pose.relative_pose.pose.orientation.w]
    q = quaternion_multiply(q1, q0)

    abs_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    return_pose = RelToAbsoluteResponse()
    return_pose.absolute_pose.pose=abs_pose
    return_pose.absolute_pose.header.frame_id = 'base_footprint'
    print(return_pose)
    return return_pose


if __name__ == '__main__':

    rospy.init_node('GetAbsolutePose')

    absolute_service = rospy.Service("/sofar/rel_to_absolute_pose",RelToAbsolute,compute_absolute_pose)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo("Service ready.")
    rospy.spin()