#!/usr/bin/env python


#	The purpose of this program is to
#	set the robot in a working position,
#	that is to say different from (0,0,0,0,0).

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def starting_pose():
	rospy.sleep(20)
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_to_starting_pose',
		              anonymous=True)
		              
	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	group = moveit_commander.MoveGroupCommander("arm_gp")

	display_trajectory_publisher = rospy.Publisher(
		                                  '/move_group/display_planned_path',
		                                  moveit_msgs.msg.DisplayTrajectory)
		                                  
	print "============ Waiting for RVIZ..."
	
	
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.x = 0.0
	pose_target.orientation.y = -1.0
	pose_target.orientation.z = 0.0
	pose_target.orientation.w = 0.0
	pose_target.position.x = 0.3
	pose_target.position.y = 0.0
	pose_target.position.z = 0.38
	group.set_pose_target(pose_target)
	
	group.go(wait=True)
	
	## When finished shut down moveit_commander.
	moveit_commander.roscpp_shutdown()
	
if __name__=='__main__':
	try:
		starting_pose()
	except rospy.ROSInterruptException:
		pass
	
	
	
