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
#include <string>
#include "math.h"
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <nodelet/nodelet.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetPlan.h"
#include <geometry_msgs/PoseStamped.h>


using namespace std;


class GraspNode{
   public:


    vector<geometry_msgs::Pose> pose_valid;
	
	//bool enable_arm;//true---left   false----right
	
	void SubCallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
	void CallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
	void pub_gripper(ros::Publisher *pub, std::string str);
	void CallBack_laserscan(const sensor_msgs::LaserScan &msg);
    void plugin_callback(const std_msgs::String::ConstPtr& msg);
  ros::NodeHandle nh_;  
  ros::AsyncSpinner spinner;
  ros::Subscriber sub;
  ros::Subscriber plugin_command_sub;
  ros::Publisher plugin_return_pub;
  ros::Subscriber laser_sub;
  ros::Publisher navigation_pub;
  ros::Publisher start_ork_pub;
  ros::Publisher stop_ork_signal_pub;
  ros::Publisher left_gripper_signal_pub;
  ros::Publisher right_gripper_signal_pub;
  ros::Publisher navigation_goal_publisher;

 

	// We will use the :planning_scene_interface:`PlanningSceneInterface
	// class to deal directly with the world.
	//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher;
	//moveit_msgs::DisplayTrajectory display_trajectory;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	
	

  GraspNode( const ros::NodeHandle &nh);
  ~GraspNode();
  void power(); 
  void init(); 
  bool navigation();
  bool detect();
  bool arrive_plan();
  bool arrive_execute();
  bool pick_plan();
  bool pick_execute();
  bool reset();
  bool wave();
  void pick_water();
  void pick_coke();
  void clear_scene();
  int  main(int argc, char **argv);
 
};
