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
class Detect{
    public:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    Detect(const ros::NodeHandle &nh);
    void Callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);


};
Detect::Detect(const ros::NodeHandle &nh){
  sub =nh_.subscribe("recognized_object_array", 1000, &Detect::Callback,this);
}
void Detect::Callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg){
   //vector<geometry_msgs::Pose> pose_valid;  
   //printf("pose_valid_max_size:%d",pose_valid.max_size());
   int current_count,listen_times,min_object_num;
   bool isReceived;
   ros::NodeHandle nh_;
   nh_.param("/grasp_demo/current_count",current_count,0);
   nh_.param("/grasp_demo/listen_times",listen_times,3);
   nh_.param("/grasp_demo/isReceived",isReceived,false);
   nh_.param("/grasp_demo/min_object_num",min_object_num,2);
   if(&(msg->objects[0])!=NULL) {
      current_count++;

      if(current_count<=listen_times){
      //ROS_INFO("I hear the pose!");
      int object_num = msg->objects.size();
      if(object_num<min_object_num) min_object_num = object_num;
      ROS_INFO("Total number of recognized objects:%d",object_num);
      for(int num=0;num<object_num;num++){
         geometry_msgs::Pose pose_init=msg->objects[num].pose.pose.pose;
         //printf("%f\n",msg->objects[0].pose.pose.pose.position.x);
         tf::StampedTransform transform;
         tf::TransformListener listener;
         try {
            listener.waitForTransform("/kinect2_rgb_optical_frame", "/base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/kinect2_rgb_optical_frame", "/base_link", ros::Time(0), transform);
          } catch (tf::TransformException ex) {
              ROS_ERROR("%s",ex.what());
          }
      
          tf::Transform transform1 = transform.inverse(), transform2(tf::Quaternion(pose_init.orientation.x, pose_init.orientation.y, pose_init.orientation.z, pose_init.orientation.w), tf::Vector3(pose_init.position.x, pose_init.position.y, pose_init.position.z));
          transform2 = transform1 * transform2;
      /*target_pose1.position=pose_init.position;
      target_pose1.orientation=pose_init.orientation;*/
         geometry_msgs::Pose target_pose1;
         target_pose1.position.x = transform2.getOrigin().getX();
         target_pose1.position.y = transform2.getOrigin().getY();
         target_pose1.position.z = transform2.getOrigin().getZ();
         target_pose1.orientation.x = transform2.getRotation().getX();
         target_pose1.orientation.y = transform2.getRotation().getY();
         target_pose1.orientation.z = transform2.getRotation().getZ();
         target_pose1.orientation.w = transform2.getRotation().getW();


     printf("sample %d:\npose:\nx:%f\ty:%f\tz:%f\n",current_count, target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
     printf("orientation:\nx:%f\ty:%f\tz:%f\tw:%f\n",target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);

      if(target_pose1.position.x<1.0&&target_pose1.position.x>0.7&&target_pose1.position.y>-0.28&&target_pose1.position.y<0.28){
        //pose_samp;
          //pose_valid.push_back(target_pose1);
          //printf("pose_valid_size:%d",pose_valid.size());
      	  float coke_x,coke_y,water_x,water_y;
      	  nh_.param("/grasp_demo/coke_x",coke_x,0.8f);
      	  nh_.param("/grasp_demo/coke_y",coke_y,0.0f);
      	  nh_.param("/grasp_demo/water_x",water_x,0.8f);
      	  nh_.param("/grasp_demo/water_y",water_y,0.0f);
          printf("\n");
      	  if(target_pose1.position.y<coke_y){
      	  	nh_.setParam("/grasp_demo/coke_x",target_pose1.position.x);
      	  	nh_.setParam("/grasp_demo/coke_y",target_pose1.position.y);
      	  }
      	  if(target_pose1.position.y>water_y){
      	  	nh_.setParam("/grasp_demo/water_x",target_pose1.position.x);
      	  	nh_.setParam("/grasp_demo/water_y",target_pose1.position.y);
      	  }
        }
 
      }
     }
      else{
  
        //printf("pose:\nx:%f\ty:%f\tz:%f\n",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
        //printf("orientation:\nx:%f\ty:%f\tz:%f\tw:%f\n",target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w); 
       
       
       isReceived = true;
       }
      }
      nh_.setParam("/grasp_demo/current_count",current_count);
      nh_.setParam("/grasp_demo/min_object_num",min_object_num);
      nh_.setParam("/grasp_demo/isReceived",isReceived);
}
int main(int argc, char **argv){

  ros::init(argc, argv, "detect_node");
  ros::NodeHandle nh;
  Detect detect(nh);
  ros::spin(); 
  return 0;
}