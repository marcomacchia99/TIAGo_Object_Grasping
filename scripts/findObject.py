#!/usr/bin/env python3
#!/usr/bin/env python

from sensor_msgs.msg import Image
import rospy
import ros_numpy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import mediapipe as mp
from scipy.spatial.transform import Rotation as R

mp_objectron = mp.solutions.objectron
global pub_targetPose


def recognize(image):
    global rotation
    global translation1
    global seq
    global pub_targetPose
    with mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,
            min_detection_confidence=0.5,
            model_name='Cup') as objectron:
        results = objectron.process(ros_numpy.numpify(image))
        if results.detected_objects:
            print('found object!')
            messageTargetPose = Pose()
            p = results.detected_objects[0].translation
            q = R.from_matrix(results.detected_objects[0].rotation).as_quat()
            messageTargetPose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            messageTargetPose.position = Point(p[0], p[1], p[2])

            pub_targetPose.publish(messageTargetPose)


if __name__ == '__main__':

    rospy.init_node('FindObject')

    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize)
    pub_targetPose = rospy.Publisher(
        '/sofar/target_rel_pose', Pose, queue_size=1)

    rospy.spin()
