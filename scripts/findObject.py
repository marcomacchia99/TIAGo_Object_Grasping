#!/usr/bin/env python3

from sensor_msgs.msg import Image
import rospy
import ros_numpy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import mediapipe as mp
from scipy.spatial.transform import Rotation as R

mp_objectron = mp.solutions.objectron

global pub_target_rel_pose
global pub_target_rel_pose_stamped


def recognize(image):
    global pub_target_rel_pose
    global pub_target_rel_pose_stamped


    with mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,
            min_detection_confidence=0.5,
            model_name='Cup') as objectron:

        results = objectron.process(ros_numpy.numpify(image))

        if results.detected_objects:

            rospy.loginfo("Object found")

            messageTargetPose = Pose()

            p = results.detected_objects[0].translation
            print(-p[2], p[0], p[1])
            q = R.from_matrix(results.detected_objects[0].rotation).as_quat()
            
            messageTargetPose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            messageTargetPose.position = Point(-p[2], p[0], p[1])

            pub_target_rel_pose.publish(messageTargetPose)

            pose_stamped = PoseStamped(pose = messageTargetPose)
            pose_stamped.header.frame_id='xtion_rgb_frame'

            pub_target_rel_pose_stamped.publish(pose_stamped)


if __name__ == '__main__':

    rospy.init_node('FindObject')

    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize)

    pub_target_rel_pose = rospy.Publisher(
        '/sofar/target_pose/relative', Pose, queue_size=1)

    pub_target_rel_pose_stamped = rospy.Publisher(
        '/sofar/target_pose/relative/stamped', PoseStamped, queue_size=1)    

    rospy.spin()
