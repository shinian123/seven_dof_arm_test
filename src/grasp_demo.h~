
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
#include <geometry_msgs/PoseStamped.h>


#define middle_state_count 1
#define LEFT_ARM  true
#define RIGHT_ARM false 
#define pi 3.141592653

using namespace std;

class Listener{
 public:
  int listen_times;
  int current_count;
  bool isReceived;
  vector<geometry_msgs::Pose> pose_sample;
 geometry_msgs::Pose pose_ans;
  void CallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg); 
  geometry_msgs::Pose average_pose(vector<geometry_msgs::Pose> pose1, int count);
  geometry_msgs::Pose add_pose(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);
  int main(int argc, char **argv);
  Listener():listen_times(2),current_count(0),isReceived(false){};
  ~Listener(){};
};
class GraspNode{
   private:
	int current_count;
	int listen_times;
	geometry_msgs::Pose target_pose1;
	bool isReceived;
	geometry_msgs::Pose pose_average;
	vector<geometry_msgs::Pose> pose_sample;
	bool enable_arm;//true---left   false----right
	void decide_target_pose(geometry_msgs::Pose *target_pose,double pose_x,double pose_y,double pose_z,double orientation_x,double orientation_y,double orientation_z,double orientation_w);
	geometry_msgs::Pose add_pose(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);
	void power(); 
	void pose_init_zero(geometry_msgs::Pose *pose); 
	void decide_orientation(geometry_msgs::Pose *target_pose2,geometry_msgs::Pose pose_average);
	geometry_msgs::Pose average_pose(vector<geometry_msgs::Pose> pose1, int count);
	void middleStateDecision(geometry_msgs::Pose pose_terminal, geometry_msgs::Pose *pose_middle);
	void SubCallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
	bool ourplan(moveit::planning_interface::MoveGroup *group,geometry_msgs::Pose target_pose2,moveit::planning_interface::MoveGroup::Plan *my_plan);
	void pub_gripper(ros::Publisher *pub, std::string str);
	char isBegin;
  char isGrasp;
  char porg;
  char lorr;
  char haveGrasp;
  Listener lis;
  std::string arm_name;
  std::string gripper_command;
  ros::NodeHandle nh,nh1;  
  ros::AsyncSpinner spinner;
  ros::Subscriber sub;
  ros::Publisher stop_ork_signal_pub;
  ros::Publisher left_gripper_signal_pub;
  ros::Publisher right_gripper_signal_pub;
  ros::Publisher navigation_goal_publisher
  moveit::planning_interface::MoveGroup group;
  

	// We will use the :planning_scene_interface:`PlanningSceneInterface
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher;
	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	
	geometry_msgs::Pose target_pose2,target_pose_temp;

 public:
  GraspNode(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &nh1 = ros::NodeHandle("~")):
    current_count(0),listen_times(2),isReceived(false),enable_arm(true),nh(nh),nh1(nh1),gripper_command("a"),spinner(1),group("left_arm")
    {}

  void init(); 
  bool navigation();
  bool detect(double &x,double &y,double &z);
  bool execute();
  int  main(int argc, char **argv);
 
};
