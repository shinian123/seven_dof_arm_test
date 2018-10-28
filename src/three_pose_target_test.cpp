/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "stdio.h"
#include "ros/ros.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Point.h"
#include <vector>
using namespace std;


geometry_msgs::Pose target_pose1;


int main(int argc, char **argv){
	ros::init(argc, argv, "grasp_node");
 	ros::NodeHandle node_handle,nh;  
 	ros::AsyncSpinner spinner(1);
 	spinner.start();

 	sleep(2.0);

 	moveit::planning_interface::MoveGroup group("left_arm");
        group.setPlannerId("RRTstarkConfigDefault");
 	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit::planning_interface::MoveGroup::Plan my_plan;

	//first pose target
	target_pose1.position.x = 0.748422;
	target_pose1.position.y = 0.009253;
	target_pose1.position.z = 0.542553;
	target_pose1.orientation.x = -0.001;
	target_pose1.orientation.y = -0.707;
	target_pose1.orientation.z = 0.707;
	target_pose1.orientation.w = 0.0;

	ROS_INFO("Begin Planning!");

	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose1);
	bool success;
	bool ex; 
	success = group.plan(my_plan);
	if(success){
		ex = group.execute(my_plan);
		if(!ex)
			ROS_INFO("first exe failed!");
	}

	ROS_INFO("\nFinish first planning!");

	sleep(3.0);

	//second pose target
	target_pose1.position.x = 0.864536;
	target_pose1.position.y = 0.367802;
	target_pose1.position.z = 0.945851;
	target_pose1.orientation.x = 0.251804;
	target_pose1.orientation.y = 0.697385;
	target_pose1.orientation.z = 0.593748;
	target_pose1.orientation.w = -0.312589;

	ROS_INFO("Begin Planning!");

	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose1); 
	success = group.plan(my_plan);
	if(success){
		ex = group.execute(my_plan);
		if(!ex){
			ROS_INFO("exe failed!");
		}
	}

	ROS_INFO("\nFinish second planning!");

	sleep(3.0);

	//reset left arm

	std::vector<double> group_variable_values;

	group_variable_values.push_back(0.1412);
	group_variable_values.push_back(-3.1415);
	group_variable_values.push_back(2.8238);
	group_variable_values.push_back(-2.8238);
	group_variable_values.push_back(-1.612);
	group_variable_values.push_back(-1.5708);
	group.setStartState(*group.getCurrentState());
	group.setJointValueTarget(group_variable_values);

	bool reset_success = group.plan(my_plan);

	if(reset_success){
		bool hui_exc=group.execute(my_plan);
		if(!hui_exc)
			ROS_INFO("Reset failed!");
	}

	ros::shutdown();  

	return 0;
}
