#!/usr/bin/python

# Send joint values to UR3 using messages

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

# waypoints = [[0,0,0,-1.5,-1.5,0], [0,0,0,0,0,0]] #surg
# waypoints = [[0,-1.57,0,-1.57,0,0], [0,0,0,0,0,0]] #bow
waypoints = [[ -0.08471905281137992, -0.20687835805975485,0.5663638322348605, -1.9445044017461406, -1.0427860228086452, -0.048257056580383484], [0.055035623401773925, -0.01403314840714387,0.12058728729265145 , -1.6726743623599312, -1.9883946406685657, 0.10691118817473733]] #bow




def main():
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR3
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    rate = rospy.Rate(1)
    cnt = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        cnt += 1

        if cnt%2 == 1:
            pts.positions = waypoints[0]
        else:
            pts.positions = waypoints[1]

        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
        
# END