#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>


class ControllerClass {
public:
  
  int current_mode = 0;
  bool goal_is_defined = false;
  static geometry_msgs::Twist velFromTeleop; // Velocity sent over from teleop_twist_keyboard
  double x_goal;
  double y_goal;
  std::string GoalID;
  
  ControllerClass();
  virtual ~ControllerClass();

private:
  // ROS NodeHandle
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle2;
  ros::NodeHandle node_handle3;

  // reate Assync spiner
  ros::AsyncSpinner spinner;
  //ACTION CLIENT
 //Prepare the goal to be sent
 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

   		
  // SUBSCRIBERS
  ros::Subscriber subMode;
  ros::Subscriber subCmdVelRemapped;
  ros::Subscriber subScanner;
  
  //SERVICES
  ros::ServiceServer service_mode;
  ros::ServiceServer service_goal;  

  void Mode_Callback(const std_msgs::Int32& msg);
  void LaserScanParserCallBack(const sensor_msgs::LaserScan::ConstPtr& scaninfo);
  void UserDriveCallBack(const geometry_msgs::Twist& msg);
  void init_param();
  void collisionAvoidance();
  void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
  
};

#endif 
