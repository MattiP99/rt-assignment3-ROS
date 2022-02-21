#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <string>
#include <termios.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"


class ControllerClass {
public:
  
  
  
  ControllerClass();
  ~ControllerClass();
  void timeoutTimerCallback(const ros::TimerEvent& event);
  void sendInfo(std::string msg);
  void LaserScanParserCallBack(const sensor_msgs::LaserScan::ConstPtr& scaninfo);
  void UserDriveCallBack(const geometry_msgs::Twist::ConstPtr& msg);
  void CancelCallBack(const std_msgs::String& msg);
  void init_param();
  void collisionAvoidance();
  void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
  bool switch_mode(final_assignment::Behavior_mode_service::Request  &req, final_assignment::Behavior_mode_service::Response &res);
  bool set_goal(final_assignment::Goal_service::Request  &req, final_assignment::Goal_service::Response &res);

private:
  // ROS NodeHandle
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle2;
  ros::NodeHandle node_handle3;

  // reate Assync spiner
  ros::AsyncSpinner spinner2;
  ros::AsyncSpinner spinner3;
  
  ros::CallbackQueue secondQueue;
  ros::CallbackQueue thirdQueue;
  //ACTION CLIENT
 //Prepare the goal to be sent
 //typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

  //PUBLISHER
  ros::Publisher pubStateInfo;
  ros::Publisher pubCmdVel;
   		
  // SUBSCRIBERS
  ros::Subscriber subMode;
  ros::Subscriber subCmdVelRemapped;
  ros::Subscriber subScanner;
 
  
  //SERVICES
  ros::ServiceServer service_mode;
  ros::ServiceServer service_goal;  
  
  int current_mode;
  bool goal_is_defined; 
  double x_goal;
  double y_goal;
  std::string GoalID;
  geometry_msgs::Twist velFromTeleop; // Velocity sent over from teleop_twist_keyboard

  
  
};

#endif 
