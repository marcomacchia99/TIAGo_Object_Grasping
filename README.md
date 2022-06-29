# Sofar Assignment - [Software architecture for Robotics (2021-2022)](https://corsi.unige.it/off.f/2021/ins/51197) , [Robotics Engineering](https://courses.unige.it/10635).
Tiago Robot simulation using ROS, Rviz and Gazebo.
================================


Introduction <img src= "https://cdn-icons.flaticon.com/png/512/3273/premium/3273644.png?token=exp=1656069884~hmac=832ed0f5cad904d64c10fc23759c2b11" width=40 height=40>
------------

>The purpose of this assignment is to develop a software architecture that uses opensource 3D object detection models to allow a robot manipulator to independently estimate the position of a given object (through an RGB-D camera) and potentially grasp it.
In order to verify the simulation in a real environment, given the availability of the Tiago robot in the laboratory, the latter mentioned was selected to carry out the software simulation on Gazebo.


Installing and Running <img src="https://media3.giphy.com/media/LwBuVHh34nnCPWRSzB/giphy.gif?cid=ecf05e47t4j9mb7l8j1vzdc76i2453rexlnv7iye9d4wfdep&rid=giphy.gif&ct=s" width="50"></h2>
--------

The simulation is built on the [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) platform, specifically the MELODIC version to be able to have the Tiago Ros package in it. Here the guide for Tiago installation . [Tiago robot](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS) 
To use the aforementioned version it was necessary to work for the project on __Ubuntu 18__ which can be donwoloaded at [Download Ubuntu 18](https://releases.ubuntu.com/18.04/).

The program requires the installation of the following packages and tools for the specific project before it can be launched:

For the part of the project relating to object recognition, the Mediapipe library was used, thanks to which we were able to simply obtain the identification of a certain object with respect to the robot's camera, in our case we chose the recognition of a cup.
MediaPipe Objectron is a mobile real-time 3D object detection solution for everyday objects. It detects objects in 2D images, and estimates their poses through a machine learning (ML) model.
We found the tutorial and the downloadable packages here:

* [Mediapipe Objectron](https://google.github.io/mediapipe/solutions/objectron)

Another tool needed was the one concerned to the part of moving the robot and grasping the object in the correct way.
__MoveIt__ was chosen for the purpose, a system that provides the necessary trajectories for the arm of a robot to put the end effector in a given place. The wrappers provide functionality for most operations that the average user will likely need, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot.

* [MoveIt](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)
 
Easy to install with:

```bash
	$ sudo apt install ros-melodic-moveit
```

To launch the simulation, you should have to run a .launch file called:

[__assignment.launch__](https://github.com/marcomacchia99/SOFAR_Assignment/blob/master/launch/assignment.launch)

Environment <img src="https://www.generationrobots.com/blog/wp-content/uploads/2016/07/gazebo-and-ros-687x319.jpg" width="80"></h2>
--------

As soon as the simulation starts, Rviz (a 3D visualizer for the Robot Operating System (ROS) framework) and Gazebo (an open-source 3D Robotics simulator) appear on the screen. We used Rviz mainly to control the right movement of the robot's arm in free space, verify the correct position of links and frames attached to the robot and to warrant the transformation matrices were the expected ones.

As regards the Gazebo environment, on the other hand, we used it to verify the correct functioning of the code within a real simulation world.
ROS creates the environment described in the __world__ folder, regarding to this we have built an environment called `table_and_cup_world` where the robot spawns and in front of him you can find a table with a cup on it .

FOTO DEL MONDO CON ROBOT APPENA SPAWNATO (?)

Implementation choices
--------------
First of all, the purpose of the assignment was not a simple matter to deal with and we thought it was not the best idea to use a single script of code. We therefore decided to separate the things to do as much as possible to achieve the goal of having a more modular code.

Then, the approach used was the more general that we could to better obtain as a final result, a robot which can adjust its position depending on the object 
coordinates in the space (however the object must remain in the range of the robot camera).

Another thing to face up with was that the Objectron tool from Mediapipe returns the frames and coordinates of the robot camera frame, more in specific the /xtion/rgb topic, so we had to dedicate a whole node to extract the relative pose of the object with respect to the camera and with the correct transformations, thanks to tf package functions, have the pose of the object with respect to the base frame (base_footprint) of the robot.

To achieve the final change of coordinates it was necessary to pass through the quaternions as regards the orientation of the object, and then multiply them thanks to the appropriate function `quaternion_multiply` with the corresponding component of each coordinate.

Object Recognition <img src="https://media3.giphy.com/media/plpY1udWkENMUe26kK/giphy.gif?cid=ecf05e4738d4g1yng7f307lr7ek62o0tnffd1n7dullxy73c&rid=giphy.gif&ct=s" width="50"></h2>
------

It was decided to implement a single node that provided the recognition of the object as soon as it was displayed within the visual range of the camera of the Tiago robot. In order to achieve it, firstly we imported the needed libraries for our aim, such as `ros_numpy`, `mediapipe`, `sensor_msgs` to import from the robot sensors the image seen from the camera and from `geometry_msgs` the object Pose, useful to extrapolate the position with `Point` and the orientation in quaternions. 

To obtain in the right way the rotation matrix we just pass through the __scipy__ open source library that could extract and import the orientation of the object with respect to the robot camera frame.

At this point we made up the _recognize_ function that takes in input the image which is taken from the subscription to the `/xtion/rgb/image_raw` topic.
Now there is the definition of the object takes place via mediapipe as objectron, as shown below:

```python
with mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,
            min_detection_confidence=0.5,
            model_name='Cup') as objectron:
```

Then we converted the BGR image to RGB and process it with MediaPipe Objectron with `objectron.process(ros_numpy.numpify(image))` and, if the object is detected, we printed out a log message for real-time feedback.

Another matter was to use the quaternions to work properly in ROS with rotation matrices. For this reason scipy library was used, which provides the orientation as a rotation matrix and then it can be convertible to the quaternion format.
The compact command used is the following:

```python
q = R.from_matrix(results.detected_objects[0].rotation).as_quat()
```

Finally we ended up the node with the publishers to the '/sofar/target_pose/relative' and the '/sofar/target_pose/relative/stamped' for the Pose and PoseStamped topic for rispectively publish the correct relative position of the coordinate frame of the object with respect to the robot camera frame and the one used to visualize it on RViz.

Object change of coordinates to base frame
-------

As already mentioned above, the coordinates taken from the geometry_msgs topic are with respect to the camera frame of the Tiago robot. It is therefore necessary, through a transformation, to make a change of coordinates to pass to the reference system with respect to the base frame. To do this we used tf2 package to have the right transformation.

We made up the __RelToAbsolute__ service which is build with this structure:


*geometry_msgs/PoseStamped relative_pose*

*---*

*geometry_msgs/PoseStamped absolute_pose*


Then we used a specific function which gets the transform between two frames by frame ID assuming fixed frame:

```python
tfBuffer.lookup_transform('base_footprint', rel_pose.relative_pose.header.frame_id, rospy.Time())
```
Once the transformation was obtained, the `do_transform_point` function was used to pass from the cartesian cordinates with respect to the camera frame to the *base footprint* frame, corresponding to the base frame, the one attached to the base of the robot. 

As previously done, the last thing to do is to pass into the quaternion domain to work in the format that ROS wants.
This is simply made by the function `quaternion_multiply` which makes the change of coordinates multiplicating the two quaternions previously built with the coordinates components regarding the relative pose to the camera frame and the ones relative to the transformation.

In the end we have built up the final absolute pose with respect to the base frame and returned printing it.

Robot grasp
------

For the part concerning the grasping movement, two nodes have been implemented that deal specifically with that: one regarding the real final approach to the object and picking it (`pickObject.py`) and one about what the robot do when the simulation starts, so the various actions before the grab and after that (`pickClient.py`).

### Pick client

As already mentioned, this node deals with managing the movements of the robot when the simulation begins. To do this we used the SimpleActionClient library to use [PlayMotion](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion), thanks to which we were able to send a goal to reach a certain position with adequate configuration of the robot's joints and then perform a certain action; we used trajectory_msgs.msg package defines messages for defining robot trajectories, so to adjust some movements more accurately or to make the robot move specific parts, such as the head to find the object; an ApproachObject service created by us as follows:

*geometry_msgs/Pose pose*

*---*

*bool result*


### Pick object


Rqt graph
------
The project graph, which illustrates the relationships between the nodes, is shown below. Keep in mind that this is a graph built by executing all the nodes at the same time in order to obtain a full graph.

The graph can be generated using this command:
 
```console
$ rqt_graph
``` 

FOTO RQT GRAPH


Flowchart (?)
-------


Simulation
-------


Conclusions and possible future improvements
--------------


