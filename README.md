# Engineering Capstone Project - AUT-21-04348
## Robot Arm Control for Minimally Invasive Surgery

This repository demonstrates a robot arm controller system for the UR3 robotic arm. A python script was developed to control the robotic arm within a simulated environment inside Gazebo.

The following repository assumes that [ROS](https://www.ros.org/) is installed and setup on a linux machine. This package has been tested using `ROS Kinetic` running on a `Ubuntu 16.04 LTS` machine.

## Simulated Environment [Gazebo & RViz]

### Planned Environment Layout:
<img src="https://github.com/kyleprr/Capstone-Project-Surgical-Robot/blob/main/media/planned-environment-layout.jpg" width="850">

### Final Environment Layout (Due to hardware issues):
<img src="https://github.com/kyleprr/Capstone-Project-Surgical-Robot/blob/main/media/Simulation-full.gif" width="850">


## Constrol System
1. [`surgical_robot_controller.py`](https://github.com/kyleprr/Capstone-Project-Surgical-Robot/blob/main/surgical_robot/scripts/surgical_robot_controller.py) publishes waypoints for the robot arm to follow in sequence to perform the required surgery.

2. [`move_robot_arm.py`](https://github.com/kyleprr/Capstone-Project-Surgical-Robot/blob/main/surgical_robot/scripts/move_robot_arm.py) is not part of the project but a short script to test the movement of the robotic arm to ensure it has been calibrated correctly.

3. A short video has been put together to show how to run the launch file and control system: https://youtu.be/LHtCcleCMA4

4. A useful tool to visualise the UR robot movements can be found here: https://cyberbotics.com/doc/guide/ure


## How to use this repository
- This project was developed and tested in [Ubuntu 16.04 LTS (Xenial Xerus)](https://releases.ubuntu.com/16.04/) with `ROS Kinetic`.
- Make sure you have installed [Python2.7](https://www.python.org/download/releases/2.7/) and the required packages & libraries listed below:

Install Universal Robots ROS Packages:
```
  $ sudo apt-get install ros-kinetic-universal-robot
  $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git universal_robots_ros_driver
  $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git universal_robot
  ```

Install Universal Robots ROS Drivers:
```
  $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git universal_robots_ros_driver
  $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git universal_robot
  ```

- Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), Gazebo, RViz, Universal Robots and MoveIt packages. 
- Created a workspace named as `surgical_robot_ws`, download the repository to `surgical_robot_ws/src/`
  ```
  $ cd surgical_robot_ws/src
  $ git clone https://github.com/kyleprr/Capstone-Project-Surgical-Robot.git
  ```
- Build the code under directory `surgical_robot_ws/`,
  ```
  $ catkin build
  $ source devel/setup.bash
  ```
- Run the code with ROS, Gazebo & RViz
  ```
  $ roslaunch surgical_robot surgical_UR3.launch
  ```
  
- Run the controller code (~/surgical_robot_ws/src/surgical_robot/scripts)
  ```
  $ python surgical_robot_controller.py
  ```