# Sofar Assignment - [Software architecture for Robotics (2021-2022)](https://corsi.unige.it/off.f/2021/ins/51197) , [Robotics Engineering](https://courses.unige.it/10635).
Tiago Robot simulation using ROS, Rviz and Gazebo.
================================


Introduction <img src= "https://cdn-icons.flaticon.com/png/512/3273/premium/3273644.png?token=exp=1656069884~hmac=832ed0f5cad904d64c10fc23759c2b11" width=40 height=40>
------------

>The purpose of this assignment is to develop a software architecture that uses opensource 3D object detection models to allow a robot manipulator to independently estimate the position 
of a given object (through an RGB-D camera) and potentially grasp it.


Installing and Running <img src="https://media3.giphy.com/media/LwBuVHh34nnCPWRSzB/giphy.gif?cid=ecf05e47t4j9mb7l8j1vzdc76i2453rexlnv7iye9d4wfdep&rid=giphy.gif&ct=s" width="50"></h2>
--------

The simulatotion is built on the [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) platform, specifically the MELODIC version.
To be able to use the aforementioned version it was necessary to work for the project on __Ubuntu 18__ which can be donwoloaded at [Download Ubuntu 18](https://releases.ubuntu.com/18.04/).

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

Environment <img src="https://www.generationrobots.com/blog/wp-content/uploads/2016/07/gazebo-and-ros-687x319.jpg" width="20"></h2>
--------
