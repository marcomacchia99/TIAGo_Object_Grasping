#!/usr/bin/env python3

import geometry_msgs.msg
import tf2_ros
from tf.transformations import *
from  tf2_geometry_msgs import *
import math
import ros_numpy
from sensor_msgs.msg import Image
from cv2 import rotatedRectangleIntersection
import rospy
from geometry_msgs.msg import Twist
import mediapipe as mp
mp_objectron = mp.solutions.objectron

global rotation
global translation1
global seq 


def speed():
    pub_vel = rospy.Publisher(
        '/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        velocity.linear.x = 0.5
        pub_vel.publish(velocity)
        rate.sleep()


def recognize(image):
    global rotation
    global translation1
    global seq
    with mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,
            min_detection_confidence=0.5,
            model_name='Cup') as objectron:

        results = objectron.process(ros_numpy.numpify(image))
        if results.detected_objects:
            sub_camera.unregister()
            print('found object!')
            rotation = results.rotation
            translation1 = results.translation
            trans = tfBuffer.lookup_transform('/xtion_optical_frame', '/base_footprint', rospy.Time())
            point = geometry_msgs.msg.PointStamped()
            point.header.frame_id='/xtion_optical_frame'
            point.header.seq=seq
            seq = seq + 1
            point.header.stamp = rospy.Time()
            point.point.x=translation1.x
            point.point.y=translation1.y
            point.point.z=translation1.z
            pointTransformed = do_transform_point(point, trans.transform)
            q = quaternion_from_matrix(rotation)
            final_quaternion = trans.transform.rotation*q


if __name__ == '__main__':
    seq=1
    rospy.init_node('Assignment')
    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rospy.spin()
