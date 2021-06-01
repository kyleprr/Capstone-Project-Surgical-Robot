# Engineering Capstone Project - AUT-21-04348
## Robot Arm Control for Minimally Invasive Surgery

This repository demonstrates a surgical robot controller system for the UR3 robotic arm. A python script was developed to control the robotic arm.


[`surgical_robot_controller.py`](https://github.com/kyleprr/Capstone-Project-Surgical-Robot/blob/main/surgical_robot/scripts/surgical_robot_controller.py) controls the robot.

[`move_robot_arm.py`](https://github.com/kyleprr/Capstone-Project-Surgical-Robot/blob/main/surgical_robot/scripts/move_robot_arm.py) move the robot between two points.




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
