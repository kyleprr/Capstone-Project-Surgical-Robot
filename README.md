# Engineering Capstone Project - AUT-21-04348
## Robot Arm Control for Minimally Invasive Surgery

This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a USB cam to detect a red box on a conveyor ([`ur5_vision.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_vision.py)), and publish its position. UR5 plans its motion ([`ur5_mp.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_mp.py)) to follow the box. Once the end-effector gets close enough to the box, it approaches the box with vacuum grippers turning on ([`ur5_gripper.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_gripper.py)). Since the vacuum gripper only provides limited force, we placed multiple grippers in order to lift the object. 

#### How to use this repository
- This project was developed and tested in Ubuntu 16.04 LTS (Xen) with ROS Kinetic.
- Make sure you have installed Python2.7 and the required packages & libraries.
- Install ROS kinetic, Gazebo, RViz, Universal Robots and MoveIt. 
- Assuming your universal robot workspace is named as `surgical_robot_ws`, download the repository to `surgical_robot_ws/src/`
  ```
  $ cd ur_ws/src
  $ git clone https://github.com/kyleprr/Capstone-Project-Surgical-Robot.git
  ```
- Build the code under directory `surgical_robot_ws/`,
  ```
  $ catkin_make
  $ source devel/setup.bash  
  ```
- Run the code with ROS, Gazebo & RViz
  ```
  $ roslaunch surgical_robot surgical_UR3.launch
  ```
  
- Run the controller code
  ```
  $ python surgical_robot_controller.py
  ```
