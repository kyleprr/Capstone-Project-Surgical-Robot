#!/usr/bin/env python

# Modified by Kyle Pereira for Surgical Capstone Project Autumn 2021

# Sources Used:
# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# MoveIt Tutorial scripts

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman


## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:

"""
Terminal Output Format: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
Code Required Format: ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

    ## Initiate a `RobotCommander` object. This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()

    ## Initiate a `PlanningSceneInterface` object. This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Initiate a `MoveGroupCommander` object.  This object is an interface to one group of joints.
    ## This interface can be used to plan and execute motions on the robot:
    group = moveit_commander.MoveGroupCommander("manipulator") ## UR3 robot

    ## We create a `DisplayTrajectory` publisher which is used to publish trajectories for RViz to visualise:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    ## Getting Basic Robot Information
    # Get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "Reference frame is: %s" % planning_frame

    # Print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "End effector: %s" % eef_link

    # Get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "Robot Groups:", robot.get_group_names()

    # Robot State:
    print "Robot State:"
    print robot.get_current_state()
    print ""

    # Variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


### BEGIN WAYPOINTS ###

  def go_to_joint_state_up(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [8.13E-05,-1.570742081,3.51E-05,-1.570706799,-1.87E-05,9.63E-05]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_pick_t1(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.44E+00,0.095875728,6.54E-02,-1.801468725,-1.51E+00,7.37E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_incision_1(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [-1.48E-02,-0.187285299,5.18E-01,-1.787017385,-1.55E+00,-1.78E-03]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_incision_2(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [7.21E-02,-0.063394436,2.44E-01,-1.639755506,-1.54E+00,8.47E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_drop_t1(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.44E+00,0.095875728,6.54E-02,-1.801468725,-1.51E+00,7.37E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_pick_t2(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.30E+00,0.405495506,5.63E-02,-2.030675439,-1.61E+00,-1.72E+00]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_surg(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [-1.35E-02,-0.170394006,4.43E-01,-1.88332408,-1.53E+00,2.93E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_surg_left(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [-8.47E-02,-0.206878358,5.66E-01,-1.944504402,-1.04E+00,-4.83E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_surg_right(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [5.50E-02,-0.014033148,1.21E-01,-1.672674362,-1.99E+00,1.07E-01]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_surg_up(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [-4.02E-03,-0.201215739,7.42E-01,-2.589742394,-1.59E+00,3.45E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_surg_down(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [-1.04E-02,-0.053113435,1.42E-01,-1.580073414,-1.57E+00,3.23E-02]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_init(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [0.35687346138767584, -0.3172340910461777, -0.24038239588294097, -1.541959949374295, -1.642023194088587, 1.9420099164006128]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

### END WAYPOINTS ###

### MAIN CONTROLLER ###

def main():
  try:
    print "Press 'ENTER' to begin. (CTRL + D) to exit:"
    raw_input() # Wait for Enter or CTRL + D
    controller = MoveGroupPythonIntefaceTutorial() # Obatin basic robot information

    print "1 - Up Position"
    controller.go_to_joint_state_up() # Move to up position

    print "2 - Pick Up Surgical Knife"
    controller.go_to_joint_state_pick_t1() # Pick up Surgical Knife
    controller.go_to_joint_state_init() # Init

    print "3 - Move to body and make incision cuts"
    controller.go_to_joint_state_incision_1()
    controller.go_to_joint_state_incision_2()
    controller.go_to_joint_state_incision_1()
    controller.go_to_joint_state_incision_2()
    controller.go_to_joint_state_incision_1()
    controller.go_to_joint_state_incision_2()
    controller.go_to_joint_state_init() # Init

    print "5 - Keep Surgical Knife back"
    controller.go_to_joint_state_drop_t1() # Keep knife back

    print "6 - Pick up Surgical tool"
    controller.go_to_joint_state_pick_t2() # Pick up new tool
    controller.go_to_joint_state_init() # Init

    print "7 - Perform Operation"
    controller.go_to_joint_state_surg()
    controller.go_to_joint_state_surg_up()
    controller.go_to_joint_state_surg_left()
    controller.go_to_joint_state_surg_down()
    controller.go_to_joint_surg_right()
    controller.go_to_joint_state_surg_up()
    controller.go_to_joint_state_surg_left()
    controller.go_to_joint_state_surg_down()
    controller.go_to_joint_surg_right()
    controller.go_to_joint_state_surg_up()
    controller.go_to_joint_state_surg_left()
    controller.go_to_joint_state_surg_down()
    controller.go_to_joint_surg_right()
    controller.go_to_joint_state_surg_up()
    controller.go_to_joint_state_surg_left()
    controller.go_to_joint_state_surg_down()
    controller.go_to_joint_surg_right()
    controller.go_to_joint_state_init() # Init

    print "8 - Keep Surgical tool back"
    controller.go_to_joint_state_pick_t2() # Keep tool back

    print "9 - Return to Up Position"
    controller.go_to_joint_state_up() # return to up position
    
    print "Operation Complete!" # End of Operation

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

  ### END OF CODE ###