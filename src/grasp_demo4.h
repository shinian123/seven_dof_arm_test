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
const char NAVIGATION1 = 3;
const char ARRIVEPLAN = 4;
const char ARRIVEEXECUTE = 5;
const char PICKPLAN = 6;
const char PICKEXECUTE = 7;
const char PLACEPLAN = 8;
const char PLACEEXECUTE = 9;
const char WATER = 10;
const char RESET = 11;
const char POWER = 12;
const char WAVE = 13;
const char COKE = 14;
const char CLEARSCENE = 15;

const std::string laser_topic = "scan";
const float MIN_TOLERANT_RANGE = 0.2f;
const float MOVE_STEP_PROP = 0.5f;
const float MIN_MOVE_STEP = MIN_TOLERANT_RANGE * 0.2;
const float SLEEP_INTERVAL = 0.3f;
using namespace std;


class GraspNode{
   public:
	int current_count;
	int listen_times;
	geometry_msgs::Pose target_pose1;
	bool isReceived;
    bool arm;
	geometry_msgs::Pose pose_average,pose_water,pose_coke;
	bool enable_arm;//true---left   false----right

	void decide_target_pose(geometry_msgs::Pose *target_pose,double pose_x,double pose_y,double pose_z,double orientation_x,double orientation_y,double orientation_z,double orientation_w);
	geometry_msgs::Pose add_pose(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);
  geometry_msgs::Pose average_pose(vector<geometry_msgs::Pose> pose1, int count);
	
	void SubCallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
	vector<geometry_msgs::Pose> pose_sample[40];
	geometry_msgs::Pose pose_ans[40];
	geometry_msgs::Pose pose_final[2];
	int min_object_num ;
	void CallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
	void pub_gripper(ros::Publisher *pub, std::string str);
	char whichobject;
	bool hit;
    float move_step;
    void CallBack_laserscan(const sensor_msgs::LaserScan &msg);
    void plugin_callback(const std_msgs::String::ConstPtr& msg);
  std::string arm_name;
  std::string gripper_command;
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
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher;
	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	
	geometry_msgs::Pose target_pose2,target_pose_temp;

  GraspNode( const ros::NodeHandle &nh);
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
