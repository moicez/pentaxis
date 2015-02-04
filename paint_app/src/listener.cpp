#include "ros/ros.h"
#include <std_msgs/String.h>
//#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit/move_group_interface/move_group.h>
// Moveit includes
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
// Filter
#include <industrial_trajectory_filters/filter_base.h>
#include <industrial_trajectory_filters/uniform_sample_filter.h>
// For time parametrisation
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// C++ includes
#include <string>
#include <iostream>
#include <sstream>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%

std::vector<std::vector<geometry_msgs::Pose2D> > drawing;
std::vector<geometry_msgs::Pose2D> waypoints;
double start_height = 0.23; // 
double draw_height = 0.20;

double scale_x = 3.436e-4;
double offset_x = 0.1972;
double scale_y = 3.436e-4;//5.119e-4;//5.9288e-4;
double offset_y = -0.20409;//-0.2047;

void robotDrawer(std::vector<geometry_msgs::Pose> &r_waypoints)
{
	moveit::planning_interface::MoveGroup group("arm_gp");
	group.setPlanningTime(10.0);
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
	moveit::planning_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(r_waypoints,
                                               0.05,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
  // Adding time parametrization to the "trajectory"
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_gp");
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  rt.getRobotTrajectoryMsg(trajectory);
  
  // Filtering the "trajectory" with Uniform Sampler
  industrial_trajectory_filters::MessageAdapter t_in;
  t_in.request.trajectory = trajectory.joint_trajectory;
  industrial_trajectory_filters::MessageAdapter t_out;
  industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
  adapter.update(t_in, t_out);
  
  // Adding the "trajectory" to the plan.
	trajectory.joint_trajectory = t_out.request.trajectory;
  my_plan.trajectory_ = trajectory;
	if (fraction == 1.0) 
		group.execute(my_plan);
	else
		ROS_WARN("Could not compute the cartesian path :( "); 

}

void robotWaypointsConstructor() 
{
	std::vector<geometry_msgs::Pose> r_waypoints;
	//geometry_msgs::Pose start_pose = group.getCurrentPose().pose; //TODO chage this to a difened pose.
	geometry_msgs::Pose target_pose;
	target_pose.position.x = 0.3;
	target_pose.position.y = 0.0;
	target_pose.position.z = start_height;
	target_pose.orientation.x = 0.0;
	target_pose.orientation.y = -1.0;
	target_pose.orientation.z = 0.0;
	target_pose.orientation.w = 0.0;
	
	for(int i = 0; i < drawing.size(); i++) {
		target_pose.position.x = drawing[i].front().x;
		target_pose.position.y = drawing[i].front().y;
		target_pose.position.z = start_height;
		r_waypoints.push_back(target_pose);
		target_pose.position.z = draw_height;
		r_waypoints.push_back(target_pose);
		for(int j = 0; j < drawing[i].size(); j++) {
			target_pose.position.x = drawing[i].at(j).x;
			target_pose.position.y = drawing[i].at(j).y;
			r_waypoints.push_back(target_pose);
		}
		target_pose.position.z = start_height;
		r_waypoints.push_back(target_pose);
		/*robotDrawer(r_waypoints);
		r_waypoints.clear();*/
	}
	robotDrawer(r_waypoints);
	r_waypoints.clear();
	
}

void msgToWaypoints(std::string str) 
{
	std::string start ("start");
	std::string end ("end");
	std::string draw ("draw");
	std::string erase ("erase");
	
	if(str.compare(start) == 0) {
		waypoints.clear();
	} 
	else if(str.compare(end) == 0) {
		drawing.push_back(waypoints);
	} 
	else if(str.compare(draw) == 0) {
		if(drawing.size() > 0) {
			robotWaypointsConstructor();
			drawing.clear();
			waypoints.clear();
		}
	} else if(str.compare(erase) == 0) {
		drawing.clear();
		waypoints.clear();
	} else  {
		geometry_msgs::Pose2D pose;
		std::stringstream ss(str);
		bool conversion = false;
		if (!(ss >> pose.y)) // purposly inveting (tablet's x will be robot's y)
		{
			ROS_WARN("Didn't convert to a float");// error: didn't convert to a float
		} 
		else if (!(ss >> pose.x))
		{
				conversion = false;
				ROS_WARN("Didn't convert to a float");// error: didn't convert to a float
		} 
		else 
		{
			pose.x = scale_x*pose.x + offset_x;
			pose.y = scale_y*pose.y + offset_y;
			pose.theta = 0.0;
			waypoints.push_back(pose);
			ROS_INFO_STREAM("Pose x = " << pose.x << " , Pose y = " << pose.y);
			
		}
	}

}


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  msgToWaypoints(msg->data.c_str());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_pentaxis");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ros_paint_view", 1000, chatterCallback);
  
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  
  
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  
  
  
	ros::spin();

  return 0;
}





