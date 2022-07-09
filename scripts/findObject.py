#!/usr/bin/env python3

"""
.. module:: findObject
:platform: Linux
:synopsis: Python module for finding object
:moduleauthor: M.Macchia S.Pedrazzi M.Haji Hosseini

Subscribes to:
    /xtion/rgb/image_raw
Publishes to:
    /sofar/target_pose/relative
    /sofar/target_pose/relative/stamped

:Find Object node description:
    Allows TIAGo to find relative position of an object in the space.
"""

# Header 
from sensor_msgs.msg import Image  
import rospy
import ros_numpy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import mediapipe as mp
from scipy.spatial.transform import Rotation as R

# Define mobile real-time 3D object detection solution 
mp_objectron = mp.solutions.objectron

# publisher of target relative pose
global pub_target_rel_pose

# publisher of target relative pose stamped
global pub_target_rel_pose_stamped


def recognize(image):
    """
        recognize function that finds the object in the image taken from media pipe and gives the
        object pose (position and orientation)

    Args:
        image (msg): image taken from robot camera
    """
    # get global variables 
    global pub_target_rel_pose
    global pub_target_rel_pose_stamped


    # Use objectron to recognize the object
    with mp_objectron.Objectron(

            static_image_mode=False,
            max_num_objects=1,

            # confidence interval
            min_detection_confidence=0.5,

            # define object 
            model_name='Cup') as objectron:
        
        # Get results
        results = objectron.process(ros_numpy.numpify(image))

        if results.detected_objects:

            # generate status 
            rospy.loginfo("FindObject - Object found")

            # a representation of pose in free space, composed of position & orientation. 
            messageTargetPose = Pose()

            # if object detected substitute coordinate use result to do translation 
            # translation coordinate (z,x,y)[meter]
            p = results.detected_objects[0].translation

            # if object detected take the orientation of the object
            # *** Rotation in 3 dimensions can be represented using unit norm quaternions***.
            # quaternion coordinate (x,y,z,w) [rad]
            q = R.from_matrix(results.detected_objects[0].rotation).as_quat()

            # define orientation via quaternions
            messageTargetPose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            # define position via transformation (converting the axes from mediapipe reference to TIAGo reference)
            messageTargetPose.position = Point(-p[2], -p[0], p[1])
            
            # publish messageTargetPose on target relative position 
            pub_target_rel_pose.publish(messageTargetPose)
            
            # Create a PoseStamped message
            pose_stamped = PoseStamped(pose = messageTargetPose)
            # pose_stamped use as frame_id --> frame of rgd camera 
            pose_stamped.header.frame_id='xtion_rgb_frame'

            # publish relative postion of object on pose_stamped 
            pub_target_rel_pose_stamped.publish(pose_stamped)


if __name__ == '__main__':

    # ros node initialization --> Find Object 
    rospy.init_node('FindObject')

    # subscribe to rgb camera to take image view
    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize)

    # publish to target_pose/relative position of object detected 
    pub_target_rel_pose = rospy.Publisher(
        '/sofar/target_pose/relative', Pose, queue_size=1)

    # publish to target_pose/relative/stamped to display position of object detected in RVIZ
    pub_target_rel_pose_stamped = rospy.Publisher(
        '/sofar/target_pose/relative/stamped', PoseStamped, queue_size=1)   

    # start infinite loop until it receives a shutdown signal
    rospy.spin()
