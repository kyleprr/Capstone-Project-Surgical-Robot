
#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:

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





  def go_to_joint_state(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -1.44
    joint_goal[2] = 1.4
    joint_goal[3] = 0.6
    joint_goal[4] = 0
    joint_goal[5] = -0.33

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)




  def go_to_joint_state2(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

    

  def go_to_joint_state3(self):
    group = self.group

    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -1.44
    joint_goal[2] = 1.4
    joint_goal[3] = 0.6
    joint_goal[4] = 0
    joint_goal[5] = -0.33

    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)



def main():
  try:
    print "Press `Enter` to begin setting up the moveit_commander (press ctrl-d to exit):"
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "Press `Enter` to execute a movement using a joint state goal:"
    #raw_input()
    tutorial.go_to_joint_state()

    print "Press `Enter` to execute a movement using a joint state goal 2:"
    #raw_input()
    tutorial.go_to_joint_state2()


    print "Press `Enter` to execute a movement using a joint state goal 3:"
    #raw_input()
    tutorial.go_to_joint_state3()

    print "Operation Complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()