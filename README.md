# Engineering Capstone Project - AUT-21-04348
## Robot Arm Control for Minimally Invasive Surgery

Surgical Robot with autonomous capabilities - Engineering Capstone Project 2021 

This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a USB cam to detect a red box on a conveyor ([`ur5_vision.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_vision.py)), and publish its position. UR5 plans its motion ([`ur5_mp.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_mp.py)) to follow the box. Once the end-effector gets close enough to the box, it approaches the box with vacuum grippers turning on ([`ur5_gripper.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_gripper.py)). Since the vacuum gripper only provides limited force, we placed multiple grippers in order to lift the object. 

#### How to use this repository
- This project was tested in Ubuntu 16.04 with ROS kinetic.
- Make sure you have installed Python2.7 and some useful libraries/packages, such as Numpy, cv2, etc.
- Install ROS kinetic, Gazebo, universal robot, Moveit, RViz. 
- Assuming your universal robot workspace is named as `ur_ws`, download the repository to `ur_ws/src/`
  ```
  $ cd ur_ws/src
  $ git clone https://github.com/lihuang3/ur5_ROS-Gazebo.git
  ```
- Under `ur_ws/src`, there are two folders: one is the official `universal_robot`, and the other is `ur5_ROS-Gazebo`. Open file `ur5_joint_limited_robot.urdf.xacro` under `ur_ws/src/universal_robot/ur_description/urdf/`, and __make the following change to the joint limit:__
  ```
    shoulder_pan_lower_limit="${-2*pi}" shoulder_pan_upper_limit="${2*pi}"
  ```
- In the same directory, make a copy of `common.gazebo.xacro` and `ur5.urdf.xacro` in case of any malfunction. 
These two default files do not include camera and vacuum gripper modules. 
So we would replace these two files with customized files. 
Under directory `ur_ws/src/ur5_ROS-Gazebo/src/ur_description/`, copy `common.gazebo.xacro` and `ur5.urdf.xacro` to `ur_ws/src/universal_robot/ur_description/urdf/`.
- Build the code under directory `ur_ws/`,
  ```
  $ catkin_make
  $ source devel/setup.bash  
  ```
- Run the code with ROS and Gazebo
  ```
  $ roslaunch ur5_notebook initialize.launch 
  ```
