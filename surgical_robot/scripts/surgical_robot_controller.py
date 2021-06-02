#!/usr/bin/env python

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

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.44E+00,0.095875728,6.54E-02,-1.801468725,-1.51E+00,7.37E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

    

  def go_to_joint_state_incision_1(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [-1.48E-02,-0.187285299,5.18E-01,-1.787017385,-1.55E+00,-1.78E-03]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



  def go_to_joint_state_incision_2(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [7.21E-02,-0.063394436,2.44E-01,-1.639755506,-1.54E+00,8.47E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_drop_t1(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.44E+00,0.095875728,6.54E-02,-1.801468725,-1.51E+00,7.37E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



  def go_to_joint_state_pick_t2(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.30E+00,0.405495506,5.63E-02,-2.030675439,-1.61E+00,-1.72E+00]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_surg(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [-1.35E-02,-0.170394006,4.43E-01,-1.88332408,-1.53E+00,2.93E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_surg_left(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [-8.47E-02,-0.206878358,5.66E-01,-1.944504402,-1.04E+00,-4.83E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_surg_right(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [5.50E-02,-0.014033148,1.21E-01,-1.672674362,-1.99E+00,1.07E-01]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



  def go_to_joint_state_surg_up(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [-4.02E-03,-0.201215739,7.42E-01,-2.589742394,-1.59E+00,3.45E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



  def go_to_joint_state_surg_down(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal = [-1.04E-02,-0.053113435,1.42E-01,-1.580073414,-1.57E+00,3.23E-02]

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



def main():
  try:
    print "Press 'ENTER' to begin. (CTRL + D) to exit:"
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial() # Obatin basic robot information

    print "1 - Up"
    tutorial.go_to_joint_state_up() # move to home position

    print "2 - Pick Up Tool 1"
    tutorial.go_to_joint_state_pick_t1()

    print "3 - Move to body - Incision Start"
    tutorial.go_to_joint_incision_1()

    print "4 - Incision End"
    tutorial.go_to_joint_incision_2()

    print "5 - Drop tool 1"
    tutorial.go_to_joint_drop_t1()

    print "6 - pick up tool 2"
    tutorial.go_to_joint_pick_t2()

    print "7 - surg1"
    tutorial.go_to_joint_surg()

    print "8 - surg left"
    tutorial.go_to_joint_surg_left()

    print "9 - surg right"
    tutorial.go_to_joint_surg_right()

    print "10 - surg up"
    tutorial.go_to_joint_surg_up()

    print "11 - surg down"
    tutorial.go_to_joint_surg_down()
    


    print "Operation Complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()