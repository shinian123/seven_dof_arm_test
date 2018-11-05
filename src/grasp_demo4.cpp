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

#include "grasp_demo4.h"
GraspNode::GraspNode(const ros::NodeHandle &nh):spinner(1)
{
   spinner.start();

  stop_ork_signal_pub = nh_.advertise<std_msgs::String>("stop_ork_signal",1000);
  start_ork_pub = nh_.advertise<std_msgs::String>("start_ork_signal",1000);
  left_gripper_signal_pub = nh_.advertise<std_msgs::String>("left_gripper_signal", 1000);
  right_gripper_signal_pub = nh_.advertise<std_msgs::String>("right_gripper_signal", 1000);
  display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 100, true);
  navigation_goal_publisher = nh_.advertise <geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
  navigation_pub = nh_.advertise<geometry_msgs::Twist>("/bulldog_velocity_controller/cmd_vel", 100);
  plugin_return_pub = nh_.advertise<std_msgs::String>("plugin_return",1000);
  plugin_command_sub = nh_.subscribe("plugin_command",1000,&GraspNode::plugin_callback,this);
  sub =nh_.subscribe("recognized_object_array", 1000, &GraspNode::CallBack,this);
  laser_sub = nh_.subscribe("scan", 1000, &GraspNode::CallBack_laserscan,this);


  pub_gripper(&right_gripper_signal_pub,"a");
	
  sleep(3.0);

  pub_gripper(&left_gripper_signal_pub,"a");
	
  sleep(3.0);
  
  ros::spinOnce();
  ROS_INFO("Init suceed!");
}
GraspNode::~GraspNode(){
  pose_valid.clear();
}

void GraspNode::CallBack(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg){
   //vector<geometry_msgs::Pose> pose_valid;  
   //printf("pose_valid_max_size:%d",pose_valid.max_size());
   int current_count,listen_times,min_object_num;
   bool isReceived;
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
          pose_valid.push_back(target_pose1);
          //printf("pose_valid_size:%d",pose_valid.size());
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

void GraspNode::CallBack_laserscan(const sensor_msgs::LaserScan &msg){
   bool hit;
   float move_step;
   nh_.param("/grasp_demo/hit",hit,false);
   nh_.param("/grasp_demo/move_step",move_step,0.0f);
   if(hit)
      return ;
   ROS_INFO("LASERSCAN_RUN");
   float MIN_TOLERANT_RANGE = 0.2f;
   float MOVE_STEP_PROP = 0.5f;
   float MIN_MOVE_STEP = MIN_TOLERANT_RANGE * 0.2;
   float SLEEP_INTERVAL = 0.3f;
   float minrange = 1e10, maxrange = -1e10;
   int i = 0;
   for(float t = msg.angle_min ; t < msg.angle_max ; t += msg.angle_increment)
   {
     if(msg.range_min < msg.ranges[i] && msg.ranges[i] < msg.range_max)
     {
        if(msg.ranges[i] < minrange)
            minrange = msg.ranges[i];
        if(msg.ranges[i] > maxrange)
            maxrange = msg.ranges[i];
     }
     i++;
   }
   ROS_INFO("%d %f", hit?1:0, minrange);
   if(minrange < MIN_TOLERANT_RANGE)
     hit = true;
   else if(!hit)
   {
     move_step = (minrange - MIN_TOLERANT_RANGE) * MOVE_STEP_PROP;
     if(move_step < MIN_MOVE_STEP)
        move_step = MIN_MOVE_STEP;
     geometry_msgs::Twist m;
     m.linear.x = move_step;
     navigation_pub.publish(m);
   }
   nh_.setParam("/grasp_demo/hit",hit);
   nh_.setParam("/grasp_demo/move_step",move_step);

}

void GraspNode::plugin_callback(const std_msgs::String::ConstPtr& message){
     std::string rec = message->data;
     std_msgs::String msg;
     stringstream ss;
     ROS_INFO("I heard %s!", rec.c_str());
     if(rec=="AUTO") {
		  ROS_INFO("Navigation succeed!");	
		  if(detect()){
			 ROS_INFO("Second step of detection succeed!");
			 reset();
			 pick_water();
			 bool arrive_plan_success = arrive_plan();
			 if(arrive_plan_success){
					sleep(5.0);
					bool arrive_execute_success = arrive_execute();
					if(arrive_execute_success){
						bool pick_plan_success = pick_plan();
						if(pick_plan_success){
							 bool pick_execute_success = pick_execute();
							if(pick_execute_success){
								sleep(5.0);
								reset();
							}
						}
					}
			 }
		  }
	 } 
     if(rec=="DETECT") {
		bool detect_success = detect();
		if(detect_success){
		  ss<<"Detect succeed!";
		}else{
		  ss<<"Detect failed!";
		}
		msg.data = ss.str();
		plugin_return_pub.publish(msg);
		ros::spinOnce();
	 } 
     if(rec=="NAVIGATION1") {
		  bool navigation_success = navigation();
          if(navigation_success) ss << "Navigation succeed!";
          else ss << "Navigation failed!";
          msg.data = ss.str();
          plugin_return_pub.publish(msg);
          ROS_INFO("Navigation plugin return has been published!");
          ros::spinOnce();
	 }
     if(rec=="ARRIVEPLAN") {
		  bool plan_success = arrive_plan();
		  if(plan_success) {
			ss << "Plan succeed!";
			//state = 3;
		  }
		  else ss << "Plan failed!";
		  msg.data = ss.str();
		  plugin_return_pub.publish(msg);
		  ROS_INFO("Plan plugin return has been published!");
		  ros::spinOnce();
	 }
     if(rec=="ARRIVEEXECUTE") {
		  bool execute_success = arrive_execute();
          if(execute_success) {
            ss << "Execute succeed!";
          }
          else ss << "Execute failed!";
          msg.data = ss.str();
          plugin_return_pub.publish(msg);
          ROS_INFO("Execute plugin return has been published!");
          ros::spinOnce();
	 }
     if(rec=="PICKPLAN") {
		  bool plan_success = pick_plan();
          if(plan_success) {
            ss << "Plan succeed!";
            //state = 5;
          }
          else ss << "Plan failed!";
          msg.data = ss.str();
          plugin_return_pub.publish(msg);
          ROS_INFO("Plan plugin return has been published!");
          ros::spinOnce();
	 } 
     if(rec=="PICKEXECUTE") {
		  bool execute_success = pick_execute();
          if(execute_success) {
            ss << "Execute succeed!";
          }
          else ss << "Execute failed!";
          msg.data = ss.str();
          plugin_return_pub.publish(msg);
          ROS_INFO("Execute plugin return has been published!");
          ros::spinOnce();
	 }
     if(rec=="PLACEPLAN") {} 
     if(rec=="PLACEEXECUTE") {}
     if(rec=="WATER")  {
		 pick_water();
	 }
     if(rec=="COKE")  {
		 pick_coke();
	 }
     if(rec=="RESET"){
		  bool reset_success = reset();
          if(reset_success) ss << "Reset succeed!";
          else ss << "Reset failed!";
          msg.data = ss.str();
          plugin_return_pub.publish(msg);
          ROS_INFO("Reset plugin return has been published!");
          ros::spinOnce();
	 }
     if(rec=="POWER") {
		sleep(1.0);
		power();
		ss<<"Power succeed!";
		msg.data = ss.str();
        plugin_return_pub.publish(msg);
        ROS_INFO("Power plugin return has been published!");
        ros::spinOnce();
	 }
     if(rec=="WAVE") {
		ss<<"Shutdown succeed!";
		msg.data = ss.str();
        plugin_return_pub.publish(msg);
        ROS_INFO("Shutdown plugin return has been published!");
        ros::spinOnce();
        
	 }
     if(rec=="CLEAR_SCENE"){
		clear_scene();
        ss<<"Clear scene succeed!";
        msg.data = ss.str();
        plugin_return_pub.publish(msg);
        ROS_INFO("Clear scene plugin return has been published!");
        ros::spinOnce();
	 }   
}

void GraspNode::power()  
{  
        FILE *fp;  
        char buf[40] = {"python netcatbotharm.py"};  
        fp = popen("bash", "w");  
        if(NULL == fp)  
        {  
           perror("popen error.\n");   
        }   
 
        fputs(buf, fp);  
        pclose(fp);    
}


void GraspNode::pub_gripper(ros::Publisher *pub, std::string str){

  std_msgs::String msg;
  std::stringstream ss;
    ss << str;
    msg.data = ss.str();
    pub->publish(msg);
  ROS_INFO("gripper signal has been published!");
}

void GraspNode::pick_water(){
    nh_.setParam("/grasp_demo/object",1);
    ROS_INFO("Pick the water bottle!");
}
void GraspNode::pick_coke(){
    nh_.setParam("/grasp_demo/object",2);
    ROS_INFO("Pick the coke!");
}
bool GraspNode::detect(){
    bool isReceived=false;
    geometry_msgs::Pose pose_water,pose_coke;
    std_msgs::String msg;
    std::stringstream ss;
    ss<<"Start object recognition";
    msg.data=ss.str();
    start_ork_pub.publish(msg);
    while(true){
      nh_.param("/grasp_demo/isReceived",isReceived,false);
      sleep(2.0);
      printf("detect_isReceived:%d",isReceived);
      if(isReceived) break;
    }
    
    if(isReceived){
       int min_y,max_y;
       printf("pose_valid_size:%d",pose_valid.size());
       if(pose_valid.empty()){
	  ROS_ERROR("Recognition failed!");
          return false;
       }
       min_y = pose_valid[0].position.y;
       max_y = pose_valid[0].position.y;
       pose_water = pose_valid[0];
       pose_coke = pose_valid[0];//1--coke 0--shanquan
       if(pose_valid.size()>1){
        for(int i=0;i<pose_valid.size();i++){
          if(pose_valid[i].position.y<min_y){
             pose_coke= pose_valid[i];
           }
          if(pose_valid[i].position.y>max_y){
             pose_water= pose_valid[i];
           }
        }
       }
      pose_valid.clear();
      pose_water.position.z=0.37;
      pose_water.orientation.w =1.0;

      pose_coke.position.z=0.37;
      pose_coke.orientation.w = 1.0;
      printf("Water:\npose:\nx:%f\ty:%f\tz:%f\n", pose_water.position.x,pose_water.position.y,pose_water.position.z);
      printf("Coke:\npose:\nx:%f\ty:%f\tz:%f\n", pose_water.position.x,pose_water.position.y,pose_water.position.z);
      nh_.setParam("/grasp_demo/water_x",pose_water.position.x);
      nh_.setParam("/grasp_demo/water_y",pose_water.position.y);
      nh_.setParam("/grasp_demo/water_z",pose_water.position.z);
      nh_.setParam("/grasp_demo/coke_x",pose_coke.position.x);
      nh_.setParam("/grasp_demo/coke_y",pose_coke.position.y);
      nh_.setParam("/grasp_demo/coke_z",pose_coke.position.z);

    
    ss.str("");
    ss.clear();
    ss << "Pause object recognition";
    msg.data = ss.str();
    stop_ork_signal_pub.publish(msg);
    ROS_INFO("Pause object recognition");
    
    sleep(1.0);

     moveit_msgs::CollisionObject cylinder,box1,box2,cyl2,wall;
     std::vector<moveit_msgs::CollisionObject> collision_objects;

      cylinder.id = "cylinder";

      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.235;
      primitive.dimensions[1] = 0.035;
  //    primitive.dimensions[2] = 0.2;

      geometry_msgs::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = pose_water.position.x;
      pose.position.y = pose_water.position.y;
      pose.position.z = 0.37;

      cylinder.primitives.push_back(primitive);
      cylinder.primitive_poses.push_back(pose);
      cylinder.operation = cylinder.ADD;

      shape_msgs::SolidPrimitive cyl2_primitive;
      cyl2_primitive.type = primitive.CYLINDER;
      cyl2_primitive.dimensions.resize(3);
      cyl2_primitive.dimensions[0] = 0.125;
      cyl2_primitive.dimensions[1] = 0.035;
  //    primitive.dimensions[2] = 0.2;

      cyl2.id = "cyl2";
      geometry_msgs::Pose cyl2_pose;
      cyl2_pose.orientation.w = 1.0;
      cyl2_pose.position.x = pose_coke.position.x;
      cyl2_pose.position.y = pose_coke.position.y;
      cyl2_pose.position.z = 0.37;

      cyl2.primitives.push_back(cyl2_primitive);
      cyl2.primitive_poses.push_back(cyl2_pose);
      cyl2.operation = cyl2.ADD;
   
      

      /* The id of the object is used to identify it. */
      box1.id = "box1";

      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive2;
      primitive2.type = primitive2.BOX;
      primitive2.dimensions.resize(3);
      primitive2.dimensions[0] = 10.2;
      primitive2.dimensions[1] = 10.2;
      primitive2.dimensions[2] = 0.01;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box1_pose;
      box1_pose.orientation.w = 1.0;
      box1_pose.position.x =  0;
      box1_pose.position.y =  0;
      box1_pose.position.z =  -0.16;

      box1.primitives.push_back(primitive2);
      box1.primitive_poses.push_back(box1_pose);
      box1.operation = box1.ADD;


      box2.id = "box2";

      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive3;
      primitive3.type = primitive.BOX;
      primitive3.dimensions.resize(3);
      primitive3.dimensions[0] = 0.225;
      primitive3.dimensions[1] = 0.55;
      primitive3.dimensions[2] = 0.33;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box2_pose;
      box2_pose.orientation.w = 1.0;
      box2_pose.position.x =  0.88;
      box2_pose.position.y =  0.1;
      box2_pose.position.z =  0.13;

      box2.primitives.push_back(primitive3);
      box2.primitive_poses.push_back(box2_pose);
      box2.operation = box2.ADD;


      wall.id = "wall";

      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive4;
      primitive4.type = primitive.BOX;
      primitive4.dimensions.resize(3);
      primitive4.dimensions[0] = 3.0;
      primitive4.dimensions[1] = 0.1;
      primitive4.dimensions[2] = 3.0;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose wall_pose;
      wall_pose.orientation.w = 1.0;
      wall_pose.position.x =  0;
      wall_pose.position.y =  1.8;
      wall_pose.position.z =  0;

      wall.primitives.push_back(primitive4);
      wall.primitive_poses.push_back(wall_pose);
      wall.operation = wall.ADD;

      
      collision_objects.push_back(cylinder);
      collision_objects.push_back(box1);
      collision_objects.push_back(box2);
      collision_objects.push_back(cyl2);
      //collision_objects.push_back(wall);
      // Once all of the objects (in this case just one) have been added to the
      // vector, we tell the planning scene to add our new box
      planning_scene_interface.addCollisionObjects(collision_objects);
      ROS_INFO("Add collision objects into the world");
      sleep(2.0);
      ///group.setPlanningTime(1.0);


      // Now when we plan a trajectory it will avoid the obstacle
      //group.setStartState(*group.getCurrentState());
      isReceived = false;
      nh_.setParam("/grasp_demo/isReceived",isReceived);
      return true;
    }else
      return false; 
}

bool GraspNode::navigation(){

  /*std::string fixed_frame = "/base_link";
  std::string robot_link = "/base_link", map_link = "/map";
  tf::TransformListener tf_listener;
  sleep(1.5);
  geometry_msgs::Pose pose_init = pose_average;
  pose_init.position.x = pose_average.position.x - 0.8 ; 

  tf::StampedTransform transform;
      tf::TransformListener listener;
      try {
            listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
      
      tf::Transform transform1 = transform.inverse(), transform2(tf::Quaternion(pose_init.orientation.x, pose_init.orientation.y, pose_init.orientation.z, pose_init.orientation.w), tf::Vector3(pose_init.position.x, pose_init.position.y, pose_init.position.z));
      transform2 = transform1 * transform2;
      /*target_pose1.position=pose_init.position;
      target_pose1.orientation=pose_init.orientation;*/
     /* geometry_msgs::Pose target_pose1;
      target_pose1.position.x = transform2.getOrigin().getX();
      target_pose1.position.y = transform2.getOrigin().getY();
      target_pose1.position.z = transform2.getOrigin().getZ();
      target_pose1.orientation.x = transform2.getRotation().getX();
      target_pose1.orientation.y = transform2.getRotation().getY();
      target_pose1.orientation.z = transform2.getRotation().getZ();
      target_pose1.orientation.w = transform2.getRotation().getW();

  double target_x = target_pose1.position.x;
  double target_y = target_pose1.position.y;
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, 90.0);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(target_x, target_y, 0.0)), ros::Time::now(), map_link);
 
      geometry_msgs::PoseStamped goal;
      tf::poseStampedTFToMsg(p, goal);
      /*ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
          goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
          goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, 90);
      *///navigation_goal_publisher.publish(goal);
      /*sleep(2.0);
      ros::spinOnce();
      sleep(10.0);*/
  bool hit = false;
  ROS_INFO("Navigation Started!");
  while(ros::ok())
  {
  	nh_.getParam("/grasp_demo/hit",hit);
    if(hit)
      break;
    ros::spinOnce();
    sleep(0.3f);
    
  }
  ROS_INFO("Navigation Finished!");
  hit = false;
 
  return true;

}

bool GraspNode::arrive_plan(){
      geometry_msgs::Pose target_pose2;
      moveit::planning_interface::MoveGroup group("left_arm");
      bool ifsuccess;
      group.setNumPlanningAttempts(20);
      group.setPlanningTime(1.0);
      group.setPlannerId("RRTstarkConfigDefault");
      group.setStartState(*group.getCurrentState());
      int whichobject;
      float water_x,water_y,water_z,coke_x,coke_y,coke_z;
      nh_.getParam("/grasp_demo/object",whichobject);
      nh_.getParam("/grasp_demo/water_x",water_x);
      nh_.getParam("/grasp_demo/water_y",water_y);
      nh_.getParam("/grasp_demo/water_z",water_z);
      nh_.getParam("/grasp_demo/coke_x",coke_x);
      nh_.getParam("/grasp_demo/coke_y",coke_y);
      nh_.getParam("/grasp_demo/coke_z",coke_z);
      if(whichobject==1){
        target_pose2.position.x=water_x;
        target_pose2.position.z=water_z+0.05;
        target_pose2.position.y=water_y+0.18;
        target_pose2.orientation.x=1;
        target_pose2.orientation.y=0;
        target_pose2.orientation.z=0;
        target_pose2.orientation.w=0;
      }else{
        target_pose2.position.x=coke_x;
        target_pose2.position.z=coke_z+0.2;
        target_pose2.position.y=coke_y;
        target_pose2.orientation.x=0;
        target_pose2.orientation.y=-sqrt(2)/2;
        target_pose2.orientation.z=sqrt(2)/2;
        target_pose2.orientation.w=0;
      }
      //target_pose2 = pose_average;
      
      //leftwards:(x:0.991776 y:-0.011776 z:0.054329  w:0.115287)  downwards:(0,-sqrt(2)/2,sqrt(2)/2,0) rightwards:(0,0,0,0)
      //decide_target_pose(&target_pose2,1.002963-0.03,-0.0816710-0.03,0.391100,0,0,1,0);

      // target_pose2 = target_pose1;
      printf("pose:\nx:%f\ty:%f\tz:%f\n",target_pose2.position.x,target_pose2.position.y,target_pose2.position.z);
      printf("orientation:\nx:%f\ty:%f\tz:%f\tw:%f\n",target_pose2.orientation.x,target_pose2.orientation.y,target_pose2.orientation.z,target_pose2.orientation.w);
      ROS_INFO("Begin Planning!");
  
      // Now, we call the planner to compute the plan
       // and visualize it.
     // Note that we are just planning, not asking move_group 
     // to actually move the robot.
      int plan_times= 10;
      for (int i=0;i<plan_times;i++){
        group.setStartState(*group.getCurrentState());
        group.setPoseTarget(target_pose2);
        group.setGoalTolerance(0.1); 
        bool success = group.plan(my_plan);
        ifsuccess = success;
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCEED":"FAILED"); 
        if(success){
          bool enable_arm = true;
          bool arm = true;
          nh_.setParam("/grasp_demo/enable_arm",enable_arm);
          nh_.setParam("/grasp_demo/arm",arm);
          break;
         }
       }
      
      return ifsuccess;
      
}
bool GraspNode::arrive_execute(){
  bool arm,enable_arm;
  nh_.getParam("/grasp_demo/arm",arm);
  nh_.getParam("/grasp_demo/enable_arm",enable_arm);
  if(arm == true){
      int whichobject;
      float water_x,water_y,water_z,coke_x,coke_y,coke_z;
      geometry_msgs::Pose target_pose2;
      nh_.getParam("/grasp_demo/object",whichobject);
      nh_.getParam("/grasp_demo/water_x",water_x);
      nh_.getParam("/grasp_demo/water_y",water_y);
      nh_.getParam("/grasp_demo/water_z",water_z);
      nh_.getParam("/grasp_demo/coke_x",coke_x);
      nh_.getParam("/grasp_demo/coke_y",coke_y);
      nh_.getParam("/grasp_demo/coke_z",coke_z);
      if(whichobject==1){
        target_pose2.position.x=water_x;
        target_pose2.position.z=water_z+0.05;
        target_pose2.position.y=water_y+0.18;
        target_pose2.orientation.x=1;
        target_pose2.orientation.y=0;
        target_pose2.orientation.z=0;
        target_pose2.orientation.w=0;
      }else{
        target_pose2.position.x=coke_x;
        target_pose2.position.z=coke_z+0.2;
        target_pose2.position.y=coke_y;
        target_pose2.orientation.x=0;
        target_pose2.orientation.y=-sqrt(2)/2;
        target_pose2.orientation.z=sqrt(2)/2;
        target_pose2.orientation.w=0;
      }
    moveit::planning_interface::MoveGroup group("left_arm");
    group.setNumPlanningAttempts(20);
    group.setPlannerId("RRTstarkConfigDefault");
    group.setPlanningTime(1.0);
    bool success = group.execute(my_plan);
    if(success){
      //sleep(1.0);
      //gripper_command=75;
      //pub_gripper(&left_gripper_signal_pub,gripper_command);
      //sleep(1.0);
      ros::spinOnce();
      target_pose2.position.y -= 0.05;
      for(int i=0;i<10;i++){
	      group.setStartState(*group.getCurrentState());
	      group.setGoalTolerance(0.02);
	      group.setPoseTarget(target_pose2);
	      success = group.plan(my_plan);
	      if(success){
		 sleep(3.0);
		 bool ex = group.execute(my_plan);
		 if(!ex) {
		     printf("execute failed!");
		}
		break;
             }
      }
      
    //ROS_INFO("Press ENTER to grasp.");
    //getchar();

      pub_gripper(&left_gripper_signal_pub,"o"); 
      sleep(2.0);

      target_pose2.position.z += 0.18;
      target_pose2.position.y += 0.18;
      target_pose2.position.x -=0.10;
      
      moveit_msgs::CollisionObject attached_object;
      
      /* The header must contain a valid TF frame*/
      attached_object.header.frame_id = group.getPlanningFrame();
      /* The id of the object */
      attached_object.id = "bottle";

      int whichobject;
      float water_x,water_y,water_z,coke_x,coke_y,coke_z;
      nh_.getParam("/grasp_demo/object",whichobject);
      nh_.getParam("/grasp_demo/water_x",water_x);
      nh_.getParam("/grasp_demo/water_y",water_y);
      nh_.getParam("/grasp_demo/water_z",water_z);
      nh_.getParam("/grasp_demo/coke_x",coke_x);
      nh_.getParam("/grasp_demo/coke_y",coke_y);
      nh_.getParam("/grasp_demo/coke_z",coke_z);

      /* A default pose */
      geometry_msgs::Pose pose;
      pose.orientation.w = 1.0;
      if(whichobject==1){
      	pose.position.x = water_x;
      	pose.position.y = water_y;
      }else{
      	pose.position.x = coke_x;
      	pose.position.y = coke_y;
      }
      pose.position.z =0.325;

      /* Define a box to be attached */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.235;
      primitive.dimensions[1] = 0.035;

      attached_object.primitives.push_back(primitive);
      attached_object.primitive_poses.push_back(pose);  
      attached_object.operation = attached_object.ADD;
      std::vector<moveit_msgs::CollisionObject> collision_objects;  
      collision_objects.push_back(attached_object);  
      planning_scene_interface.addCollisionObjects(collision_objects);
      moveit_msgs::CollisionObject remove_object;
      remove_object.id = "cyl2";
      //remove_object.header.frame_id = "left_gripper_palm_link";
      remove_object.operation = remove_object.REMOVE;

      ROS_INFO("Attaching the object to the hand and removing it from the world.");
      group.attachObject(attached_object.id);
      
      for(int i=0;i<10;i++){
      group.setStartState(*group.getCurrentState());
      group.setPoseTarget(target_pose2);
      group.setGoalTolerance(0.08);
      success = group.plan(my_plan);
      if(success){
         bool ex = group.execute(my_plan);
          if(!ex) {
             printf("execute failed!");
          }
          break;
      }
      }
      
      return true;
    }else{
      
      return false;
    }
  }else{
    moveit::planning_interface::MoveGroup group("right_arm");
    bool success = group.execute(my_plan);
    if(success){
      //sleep(1.0);
      //gripper_command=75;
      //pub_gripper(&right_gripper_signal_pub,gripper_command);
      //sleep(1.0);
      ros::spinOnce();
      
      return true;
    }else{
      
      return false;
    }
  }
}

bool GraspNode::pick_plan(){
    sleep(2.0);
    bool arm,enable_arm;
    nh_.getParam("/grasp_demo/arm",arm);
    nh_.getParam("/grasp_demo/enable_arm",enable_arm);
   
    // printf("Have we grasped the object?(y/n)");
    // scanf("%s",&haveGrasp);
    std::vector<std::string> object_ids;
    object_ids.push_back("cylinder");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
    planning_scene_interface.removeCollisionObjects(object_ids);
    
    if(arm==true){
      moveit::planning_interface::MoveGroup group("left_arm");
      group.setNumPlanningAttempts(20);
      group.setPlanningTime(1.0);
      group.setPlannerId("RRTstarkConfigDefault");
	 group.setStartState(*group.getCurrentState());
      //target_pose_temp.position.x=0.864536;
      //target_pose_temp.position.y=0.367802;
      //target_pose_temp.position.z=0.945851;
      //target_pose_temp.orientation.x=0.251804;
      //target_pose_temp.orientation.y=0.697385;
      //target_pose_temp.orientation.z=0.593748;
      //target_pose_temp.orientation.w=-0.312589;
	vector<double> group_variable_values;
	group_variable_values.push_back(2.999290);
	group_variable_values.push_back(-2.236700);
	group_variable_values.push_back(-0.559131);
	group_variable_values.push_back(-2.830370);
	group_variable_values.push_back(-1.319326);
	group_variable_values.push_back(-1.608036);
      ROS_INFO("Start planning picking!");
     // group.setPoseTarget(target_pose_temp);
	group.setJointValueTarget(group_variable_values);
      bool pick_success = group.plan(my_plan);
      if(pick_success){
        //group.execute(my_plan);
        enable_arm = true;
        arm = true;
        return true;
        }else{
        
        return false;
        }
     }else{
          moveit::planning_interface::MoveGroup group("right_arm");
           group.setPlanningTime(1.0);
          //target_pose_temp.position.x=0.566997;
          //target_pose_temp.position.y=-0.435485;
          //target_pose_temp.position.z=1.055114;
          //target_pose_temp.orientation.x=0.342295;
          //target_pose_temp.orientation.y=0.593277;
          //target_pose_temp.orientation.z=0.654907;
          //target_pose_temp.orientation.w=-0.319300;
	  vector<double> group_variable_values;
	  group.setStartState(*group.getCurrentState());
	  group_variable_values.push_back(-2.815427);
	  group_variable_values.push_back(-1.499180);
	  group_variable_values.push_back(0.997918);
	  group_variable_values.push_back(0.024248);
	  group_variable_values.push_back(1.522912);
	  group_variable_values.push_back(-1.612360);
          ROS_INFO("Start planning picking!");
	  group.setJointValueTarget(group_variable_values);
          //group.setPoseTarget(target_pose_temp);
          bool pick_success = group.plan(my_plan);
          if(pick_success){
            //group.execute(my_plan);
            enable_arm = false;
            arm = false;
            return true;
          }else{
            
            return false;
          }
        }
    //group.setRandomTarget();
    
   
}
bool GraspNode::pick_execute(){
    //power();
    sleep(1.0);
    bool arm,enable_arm;
    nh_.getParam("/grasp_demo/arm",arm);
    nh_.getParam("/grasp_demo/enable_arm",enable_arm);
    if(arm==true){
      moveit::planning_interface::MoveGroup group("left_arm");
      group.setNumPlanningAttempts(20);
      group.setPlanningTime(1.0);
      group.setPlannerId("RRTstarkConfigDefault");
      bool success = group.execute(my_plan);
      if(success){
        //sleep(3.0);
        pub_gripper(&left_gripper_signal_pub,"o");
        ros::spinOnce();
        //sleep(3.0);
        return true;
      }else{
        return false;
      }
    }else{
      moveit::planning_interface::MoveGroup group("right_arm");
      bool success = group.execute(my_plan);
      if(success){
        sleep(3.0);

        pub_gripper(&right_gripper_signal_pub,"o");
        ros::spinOnce();
        //sleep(3.0);
        return true;
      }else{
        return false;
      }
    }

}
bool GraspNode::wave(){
   moveit::planning_interface::MoveGroup group("left_arm");
        group.setPlannerId("RRTstarkConfigDefault");
        group.setPlanningTime(1.0);
        group.setNumPlanningAttempts(20);
 	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 	//ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit::planning_interface::MoveGroup::Plan my_plan;
        bool success;
        bool ex;
        int times=0;
	//first pose target
	std::vector<double> group_variable_values;
        
        for(times=0;times<3;times++){
		group_variable_values.clear();
		group_variable_values.push_back(0.903638);
		group_variable_values.push_back(-2.22677);
		group_variable_values.push_back(0.494097);
		group_variable_values.push_back(-0.4074195);
		group_variable_values.push_back(1.707196);
		group_variable_values.push_back(2.9010196);
		group.setStartState(*group.getCurrentState());
		group.setJointValueTarget(group_variable_values); 
		success = group.plan(my_plan);
		if(success){
			ex = group.execute(my_plan);
			if(!ex)
				ROS_INFO("first exe failed!");
		}


		group_variable_values.clear();
		//group_variable_values.push_back(1.23378980);
		//group_variable_values.push_back(-2.73414451);
    group_variable_values.push_back(0.903638);
    group_variable_values.push_back(-2.22677);
		group_variable_values.push_back(-0.4581005);
		//group_variable_values.push_back(-2.5075467);
		//group_variable_values.push_back(-1.527659);
		//group_variable_values.push_back(-0.5171774);
		group_variable_values.push_back(-0.4074195);
		group_variable_values.push_back(1.707196);
		group_variable_values.push_back(2.9010196);
		group.setStartState(*group.getCurrentState());
		group.setJointValueTarget(group_variable_values);

		success = group.plan(my_plan);

		if(success){
		        bool ex=group.execute(my_plan);
                                               
		}

		
	}
        return true;

}
void GraspNode::clear_scene(){
        std::vector<std::string> object_ids;
        object_ids.push_back("box1");
        object_ids.push_back("box2");
    //object_ids.push_back("wall");
        object_ids.push_back("bottle");
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.removeCollisionObjects(object_ids);

}
bool GraspNode::reset(){
    ROS_INFO("Start reseting!");
	//reset pose 
	//power();
	sleep(1.0);
	bool arm,enable_arm;
    nh_.getParam("/grasp_demo/arm",arm);
    nh_.getParam("/grasp_demo/enable_arm",enable_arm);
  
	vector<double> group_variable_values;
	if(enable_arm==true){
		moveit::planning_interface::MoveGroup group("left_arm");
              group.detachObject("botttle");
              group.setNumPlanningAttempts(20);
              group.setPlanningTime(1.0);
              group.setPlannerId("RRTstarkConfigDefault");
              group.setStartState(*group.getCurrentState());
	      group_variable_values.push_back(-0.802153889);
              group_variable_values.push_back(-1.344944779);
              group_variable_values.push_back(1.707468509674);
              group_variable_values.push_back(-3.133113686);
              group_variable_values.push_back(-0.5110691);
              group_variable_values.push_back(1.346010);
		group.setJointValueTarget(group_variable_values);
		bool hui_success = group.plan(my_plan);
      	      if(hui_success){ 
      		bool hui_exc=group.execute(my_plan);
      		/*if(!hui_exc)
      		{
      			power();
			      sleep(0.5);
			      group.setStartState(*group.getCurrentState());
      			group.setJointValueTarget(group_variable_values);
      			group.plan(my_plan);
      			group.execute(my_plan);
      		}*/
      	     }
	}else{
		    moveit::planning_interface::MoveGroup group("right_arm");
		    group.setStartState(*group.getCurrentState());
		    group_variable_values.push_back(0.76583129);
                  group_variable_values.push_back(-1.760450188);
                  group_variable_values.push_back(-1.73966819);
                  group_variable_values.push_back(0.266070008);
                  group_variable_values.push_back(0.6783955097);
                  group_variable_values.push_back(1.57658851);

		group.setJointValueTarget(group_variable_values);
		bool hui_success = group.plan(my_plan);
      	if(hui_success){ 
      		bool hui_exc=group.execute(my_plan);
      		if(!hui_exc)
      		{
      			power();
			      sleep(0.5);
			      group.setStartState(*group.getCurrentState());
      			group.setJointValueTarget(group_variable_values);
      			group.plan(my_plan);
      			group.execute(my_plan);
      		}
      	}
        }
    /*std::vector<std::string> object_ids;
    object_ids.push_back("box1");
    object_ids.push_back("box2");
    //object_ids.push_back("wall");
    object_ids.push_back("bottle");
    planning_scene_interface.removeCollisionObjects(object_ids);*/
      //lis.isReceived = false;
     // lis.pose_sample.clear();
	return true;
}
int main(int argc, char **argv){

  ros::init(argc, argv, "grasp_demo4");
  ros::NodeHandle nh;
  GraspNode graspnode(nh);
  ros::spin(); 
  return 0;
}

