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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_node");
  ros::NodeHandle node_handle,nh;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(2.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("left_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  
  tf::StampedTransform transform2;
  tf::TransformListener listener;
  try {
            listener.waitForTransform("/base_link","/left_gripper_palm", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform( "/base_link","/left_gripper_palm", ros::Time(0), transform2);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
  
   target_pose1.position.x = transform2.getOrigin().getX();
target_pose1.position.y = transform2.getOrigin().getY();
target_pose1.position.z = transform2.getOrigin().getZ();
target_pose1.orientation.x = transform2.getRotation().getX();
target_pose1.orientation.y = transform2.getRotation().getY();
target_pose1.orientation.z = transform2.getRotation().getZ();
target_pose1.orientation.w = transform2.getRotation().getW();

printf("pose:\nx:%f\ty:%f\tz:%f\n",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
printf("orientation:\nx:%f\ty:%f\tz:%f\tw:%f\n",target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);

std::vector<double>group_variable_values;
group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),group_variable_values);
for(int i=0;i<group_variable_values.size();i++){
	printf("group_variable_values.push_back(%lf);\n",group_variable_values[i]);
}
  geometry_msgs::Pose target_pose2;
  double target_x,target_y,target_z;
  moveit::planning_interface::MoveGroup::Plan my_plan;
  printf("please input target position:\n");
  printf("x:\n");
  scanf("%lf",&target_x);
  printf("y:\n");
  scanf("%lf",&target_y);
  printf("z:\n");
  scanf("%lf",&target_z);  
  target_pose2.position.x=target_x;
  target_pose2.position.y=target_y;
  target_pose2.position.z=target_z;
  target_pose2.orientation.x=-0.001;
  target_pose2.orientation.y=-0.707;
  target_pose2.orientation.z=0.708;
  target_pose2.orientation.w=0.000;
  ROS_INFO("Begin Planning!");

  // Now, we call the planner to compute the plan
   // and visualize it.
 // Note that we are just planning, not asking move_group 
 // to actually move the robot.
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(target_pose2); 
  group.setPlannerId("RRTstarkConfigDefault");
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCEED":"FAILED"); 
  if(success){
     bool ex = group.execute(my_plan);
      if(!ex) {
      	printf("execute failed!");
      }
  
  }
  sleep(5.0);

   /************Pick********************/
    
    
	group.setStartState(*group.getCurrentState());
	
		group_variable_values.clear();
		group_variable_values.push_back(0.1412);
		group_variable_values.push_back(-3.1415);
		group_variable_values.push_back(2.8238);
		group_variable_values.push_back(-2.8238);
		group_variable_values.push_back(-1.612);
		group_variable_values.push_back(-1.5708);
		group.setJointValueTarget(group_variable_values);
	        group.setPlannerId("RRTkConfigDefault");
		bool hui_success = group.plan(my_plan);
		//ROS_INFO("Visualizing plan 1 (pose goal) %s",hui_success?"SUCCEED":"FAILED");
	      	if(hui_success){ 
	      		bool hui_exc=group.execute(my_plan);
	      		if(!hui_exc)
	      		{
	                  ROS_INFO("Arrive failed!");
	      		}
	      	}
	
    sleep(5.0);


  /*********reset*********/
 // group_variable_values.clear();
  /*geometry_msgs::Pose target_pose_temp;
  target_pose_temp.position.x=0.117831;
        target_pose_temp.position.y=-0.439276;
        target_pose_temp.position.z=0.572630;
        target_pose_temp.orientation.x=0.480942;
        target_pose_temp.orientation.y=0.581612;
        target_pose_temp.orientation.z=0.532653;
        target_pose_temp.orientation.w=-0.383104; 
        group.setPoseTarget(target_pose_temp); 
        bool pick_success = group.plan(my_plan);
         
        if(pick_success){ 
                bool pick_exc=group.execute(my_plan);
                if(!pick_exc)
                {
                        ROS_INFO("Pick failed!");
                }
        }

		/*target_pose_temp.position.x=0.117831;
		target_pose_temp.position.y=0.367802;
		target_pose_temp.position.z=0.945851;
		target_pose_temp.orientation.x=0.251804;
		target_pose_temp.orientation.y=0.697385;
		target_pose_temp.orientation.z=0.593748;
		target_pose_temp.orientation.w=-0.312589;
		group.setStartState(*group.getCurrentState());
		group.setPoseTarget(target_pose_temp);
		*group_variable_values.push_back(2.999290);
		group_variable_values.push_back(-2.236700);
		group_variable_values.push_back(-0.559131);
		group_variable_values.push_back(-2.830370);
		group_variable_values.push_back(-1.319326);
		group_variable_values.push_back(-1.608036);
	      ROS_INFO("Start planning picking!");
	     //
		group.setJointValueTarget(group_variable_values);*/



		//group.setRandomTarget();
		/*ROS_INFO("Start picking!");
      	//group.setPoseTarget(target_pose_temp);
      	bool pick_success = group.plan(my_plan);
	 
      	if(pick_success){ 
      		bool pick_exc=group.execute(my_plan);
      		if(!pick_exc)
      		{
      			ROS_INFO("Pick failed!");
      		}
      	}
      	sleep(5.0);
  
  group_variable_values.clear();*/
	/*group.setStartState(*group.getCurrentState());
	
	
		
		group_variable_values.push_back(0.1412);
		group_variable_values.push_back(-3.1415);
		group_variable_values.push_back(2.8238);
		group_variable_values.push_back(-2.8238);
		group_variable_values.push_back(-1.612);
		group_variable_values.push_back(-1.5708);
		group.setJointValueTarget(group_variable_values);
		group.setPlannerId("RRTkConfigDefault");
		bool reset_success = group.plan(my_plan);
		//ROS_INFO("Visualizing plan 1 (pose goal) %s",hui_success?"SUCCEED":"FAILED");
	      	if(reset_success){ 
	      		bool hui_exc=group.execute(my_plan);
	      		if(!hui_exc)
	      		{
	      			ROS_INFO("Reset failed!");
	      		}
	      	}
    group_variable_values.clear();*/ 
  /*group.setStartState(*group.getCurrentState());
	target_pose_temp.position.x=0.152698;
		target_pose_temp.position.y=0.452910;
		target_pose_temp.position.z=0.544295;
		target_pose_temp.orientation.x=0.452260;
		target_pose_temp.orientation.y=0.522986;
		target_pose_temp.orientation.z=-0.510330;
		target_pose_temp.orientation.w=0.511381;
		group.setPoseTarget(target_pose_temp);
		
		*group_variable_values.push_back(0.1412);
		group_variable_values.push_back(-3.1415);
		group_variable_values.push_back(2.8238);
		group_variable_values.push_back(-2.8238);
		group_variable_values.push_back(-1.612);
		group_variable_values.push_back(-1.5708);
		group.setJointValueTarget(group_variable_values);*/
		
		/*bool reset_success = group.plan(my_plan);
		//ROS_INFO("Visualizing plan 1 (pose goal) %s",hui_success?"SUCCEED":"FAILED");
	      	if(z:
reset_success){ 
	      		bool hui_exc=group.execute(my_plan);
	      		if(!hui_exc)
	      		{
	      			ROS_INFO("Reset failed!");
	      		}
	      	}*/
  ros::shutdown();  
  return 0;
}

