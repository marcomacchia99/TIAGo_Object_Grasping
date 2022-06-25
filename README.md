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
First of all, the purpose of the assignment was not a simple matter to deal with and to be able to use a single script of code. We therefore decided to separate the things to do as much as possible to achieve the goal of having a more modular code.

Then, the approach used, was the more general that we could, to better obtain as a final result, a robot which can adjust its position depending on the object 
coordinates in the space (however the object must remain in the range of the robot camera).

Another thing to face up with was that the Objectron tool from Mediapipe returns the frames and coordinates of the robot camera frame, more in specific the /xtion/rgb topic, so we had to dedicate a whole node to extract the relative pose of the object with respect to the camera and with the correct transformations, thanks to tf package functions, have the pose of the object with respect to the base frame (base_footprint) of the robot.

