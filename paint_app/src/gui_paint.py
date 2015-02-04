#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import wx
import sys
import rospy
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os

class Example(wx.Frame):
           
    def __init__(self, *args, **kw):
        super(Example, self).__init__(*args, **kw) 
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pentaxis_gui', anonymous=True)
        self.InitRobot()
        self.InitUI()
    
    def InitRobot(self):
    		robot = moveit_commander.RobotCommander()
    		scene = moveit_commander.PlanningSceneInterface()
    		self.armGroup = moveit_commander.MoveGroupCommander("arm_gp")
    		self.eefGroup = moveit_commander.MoveGroupCommander("eef_gp")
    		display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
        
        
    def InitUI(self):   
        pnl = wx.Panel(self)
        self.ToggleWindowStyle(wx.STAY_ON_TOP)
        drawBtn = wx.Button(pnl, label='Send Drawing to Robot', pos=(20, 20))
        workBtn = wx.Button(pnl, label='Go to Work Position', pos=(250, 20))
        drwrdyBtn = wx.Button(pnl, label='Go to Draw Ready Position', pos=(20, 60))
        performanceBtn = wx.Button(pnl, label='Precision Test', pos=(293, 60))
        #homingBtn = wx.Button(pnl, label='Homing', pos=(20, 100))
        openBtn = wx.Button(pnl, label='EEF Open', pos=(220,100))
        closeBtn = wx.Button(pnl, label='EEF Close', pos=(313, 100))
        pickPenBtn = wx.Button(pnl, label='Pick Marker', pos=(20, 180))
        placePenBtn = wx.Button(pnl, label='Place Marker', pos=(120, 180))
        FullBtn = wx.Button(pnl, label='Full Test', pos=(260, 180))
        
        
        self.paintPub = rospy.Publisher('ros_paint_view', String, queue_size=10)
        self.r = rospy.Rate(10) # 10hz

        drawBtn.Bind(wx.EVT_BUTTON, self.drawEvent)
        drwrdyBtn.Bind(wx.EVT_BUTTON, self.drawPoseEvent)
        workBtn.Bind(wx.EVT_BUTTON, self.workPoseEvent)
        openBtn.Bind(wx.EVT_BUTTON, self.openEefEvent)
        closeBtn.Bind(wx.EVT_BUTTON, self.closeEefEvent)
        pickPenBtn.Bind(wx.EVT_BUTTON, self.pickMarker)
        placePenBtn.Bind(wx.EVT_BUTTON, self.placeMarker)
        performanceBtn.Bind(wx.EVT_BUTTON, self.performEvent)
        FullBtn.Bind(wx.EVT_BUTTON, self.fulltest)

        self.SetSize((415, 230))
        self.SetTitle('CITEDI - Instituto Politécnico Nacional')
        #self.Centre()
        self.Show(True)          
        
    def drawEvent(self, e):
        str = "draw"
        rospy.loginfo(str)
        self.paintPub.publish(str)
        self.r.sleep()
    
    def drawPoseEvent(self, e):
        print "============ Generating plan: Draw Ready"
        self.armGroup.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 0.0
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = -1.0
        pose_target.orientation.z = 0.0
        pose_target.position.x = 0.3
        pose_target.position.y = 0.0
        pose_target.position.z = 0.23
        self.armGroup.set_pose_target(pose_target)
        self.armGroup.go(wait=True)
				
    def workPoseEvent(self, e):
        print "============ Generating plan: Work Ready"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        os.system("rosclean purge -y")
				
    def openEefEvent(self, e):
        print "============ Generating plan: Open EEF"
        self.eefGroup.clear_pose_targets()
        group_variable_values = self.eefGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = 0.0
        self.eefGroup.set_joint_value_target(group_variable_values)
        self.eefGroup.go(wait=True)
				
    def closeEefEvent(self, e):
        print "============ Generating plan: Close EEF"
        self.eefGroup.clear_pose_targets()
        group_variable_values = self.eefGroup.get_current_joint_values()
        group_variable_values[0] = -0.0253
        group_variable_values[1] = -0.0253
        self.eefGroup.set_joint_value_target(group_variable_values)
        self.eefGroup.go(wait=True)
				
    def pickMarker(self, e):
        print "============ Generating plan: Pick Marker"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.000294262318387
        group_variable_values[1] = 0.175985754767
        group_variable_values[2] = 1.40465564517
        group_variable_values[3] = 0.02259414992
        group_variable_values[4] = -5.58909651932e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(4)
        group_variable_values[0] = 0.000286578179123
        group_variable_values[1] = 0.408346478104
        group_variable_values[2] = 1.10858665081
        group_variable_values[3] = 0.0349353089361
        group_variable_values[4] = -4.22125861566e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(3)
        print "============ Generating plan: Close EEF"
        group_variable_values = self.eefGroup.get_current_joint_values()
        group_variable_values[0] = -0.0253
        group_variable_values[1] = -0.0253
        self.eefGroup.set_joint_value_target(group_variable_values)
        self.eefGroup.go(wait=True)
        # Go back to work position
        rospy.sleep(2)
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
				
    def placeMarker(self, e):
        print "============ Generating plan: Place Marker"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.000286578179123
        group_variable_values[1] = 0.408346478104
        group_variable_values[2] = 1.10858665081
        group_variable_values[3] = 0.0349353089361
        group_variable_values[4] = -4.22125861566e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        rospy.sleep(4)
        print "============ Generating plan: Open EEF"
        group_variable_values = self.eefGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = 0.0
        self.eefGroup.set_joint_value_target(group_variable_values)
        self.eefGroup.go(wait=True)
        # Go back to work position
        rospy.sleep(2)
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.000294262318387
        group_variable_values[1] = 0.175985754767
        group_variable_values[2] = 1.40465564517
        group_variable_values[3] = 0.02259414992
        group_variable_values[4] = -5.58909651932e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        rospy.sleep(3)
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
				
    def performEvent(self, e):
        print "============ Generating plan: Presicion Test"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.00030141533641
        group_variable_values[1] = -0.282702579197
        group_variable_values[2] = 2.02282357331
        group_variable_values[3] = 1.40210660429
        group_variable_values[4] = 2.2867037502e-07
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        rospy.sleep(4)
        print "============ Generating plan: Square"
        os.system("rosrun trajectory_tutorials square")
        rospy.sleep(24)
        print "============ Generating plan: Circle"
        os.system("rosrun trajectory_tutorials circle")
        rospy.sleep(8)
        print "============ Generating plan: Ellipse"
        os.system("rosrun trajectory_tutorials ellipse")
        rospy.sleep(8)
        print "============ Generating plan: Sine"
        os.system("rosrun trajectory_tutorials sine")
        rospy.sleep(23)
        print "============ Generating plan: Work Ready"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        
    def fulltest(self, e):
        print "============ Generating plan: Pick Marker"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.000294262318387
        group_variable_values[1] = 0.175985754767
        group_variable_values[2] = 1.40465564517
        group_variable_values[3] = 0.02259414992
        group_variable_values[4] = -5.58909651932e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(4)
        group_variable_values[0] = 0.000286578179123
        group_variable_values[1] = 0.408346478104
        group_variable_values[2] = 1.10858665081
        group_variable_values[3] = 0.0349353089361
        group_variable_values[4] = -4.22125861566e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(3)
        print "============ Generating plan: Close EEF"
        group_variable_values = self.eefGroup.get_current_joint_values()
        group_variable_values[0] = -0.0253
        group_variable_values[1] = -0.0253
        self.eefGroup.set_joint_value_target(group_variable_values)
        self.eefGroup.go(wait=True)
        # Go back to work position
        #rospy.sleep(2)
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
			  ### do figure #####
        print "============ Generating plan: Paint Position"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.00030141533641
        group_variable_values[1] = -0.282702579197
        group_variable_values[2] = 2.02282357331
        group_variable_values[3] = 1.40210660429
        group_variable_values[4] = 2.2867037502e-07
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(4)
        print "============ Generating plan: Square"
        os.system("rosrun trajectory_tutorials square")
        #rospy.sleep(16)
        print "============ Generating plan: Work_pos"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)   
        print "============ Generating plan: Place Marker"
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.000286578179123
        group_variable_values[1] = 0.408346478104
        group_variable_values[2] = 1.10858665081
        group_variable_values[3] = 0.0349353089361
        group_variable_values[4] = -4.22125861566e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(4)
        print "============ Generating plan: Open EEF"
        group_variable_values = self.eefGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = 0.0
        self.eefGroup.set_joint_value_target(group_variable_values)
        self.eefGroup.go(wait=True)
        # Go back to work position
        #rospy.sleep(2)
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.000294262318387
        group_variable_values[1] = 0.175985754767
        group_variable_values[2] = 1.40465564517
        group_variable_values[3] = 0.02259414992
        group_variable_values[4] = -5.58909651932e-05
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        #rospy.sleep(3)
        self.armGroup.clear_pose_targets()
        group_variable_values = self.armGroup.get_current_joint_values()
        group_variable_values[0] = 0.0
        group_variable_values[1] = -0.938212290489
        group_variable_values[2] = 1.8604486489
        group_variable_values[3] = 2.21935629517
        group_variable_values[4] = 0.0
        self.armGroup.set_joint_value_target(group_variable_values)
        self.armGroup.go(wait=True)
        
def main():
    
    ex = wx.App()
    Example(None)
    ex.MainLoop()    


if __name__ == '__main__':
    main()   

