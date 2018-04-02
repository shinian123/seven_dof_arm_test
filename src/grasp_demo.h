
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


#define middle_state_count 1
#define LEFT_ARM  true
#define RIGHT_ARM false 
#define pi 3.141592653
#define AUTO_MODE 1
#define DETECT_MODE 2
#define NAVIGATION_MODE 3
#define EXECUTE_MODE 4
const char AUTO = 1;
const char DETECT = 2;
const char NAVIGATION = 3;
const char EXECUTE = 4;
const char PLAN = 6;
const char RESET = 5;

const std::string laser_topic = "scan";
const float min_tolerant_range = 0.3f;
const float move_step = 0.5f;
const float sleep_interval = 0.4f;
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
  Listener():listen_times(2),current_count(0),isReceived(false){};
  ~Listener(){};
};
class pluginListener
{
public:
	int mode;
	void plugin_callback(const std_msgs::String::ConstPtr& msg);
	pluginListener():mode(0){};
	~pluginListener(){};
	
};
class laserListener
{
public:
    bool hit;
    void CallBack_laserscan(const sensor_msgs::LaserScan &msg);
    laserListener():hit(false){};
    ~laserListener(){}
    
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
  pluginListener plis;
  laserListener laserlis;
  Listener lis;
  std::string arm_name;
  std::string gripper_command;
  ros::NodeHandle nh,nh1;  
  ros::AsyncSpinner spinner;
  ros::Subscriber sub;
  ros::Subscriber plugin_command_sub;
  ros::Publisher plugin_return_pub;
  ros::Subscriber laser_sub;
  ros::Publisher navigation_pub;
  ros::Publisher stop_ork_signal_pub;
  ros::Publisher left_gripper_signal_pub;
  ros::Publisher right_gripper_signal_pub;
  ros::Publisher navigation_goal_publisher;
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
  bool plan(double x,double y,double z);
  bool execute(double x,double y,double z);
  bool reset();
  int  main(int argc, char **argv);
 
};
