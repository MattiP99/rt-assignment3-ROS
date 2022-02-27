#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <string>
#include <termios.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <boost/bind.hpp>

#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"
#include <std_srvs/SetBool.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

class ControllerClass {
public:
  
  
  
  ControllerClass(ros::NodeHandle* node_handle);
  ~ControllerClass();
 
  void sendInfo(bool temp);
  
  void UserDriveCallBack(const geometry_msgs::Twist::ConstPtr& msg);
  void CancelCallBack(const std_msgs::String& msg);
  //For driving
  void collisionAvoidance();
  void LaserScanParserCallBack(const sensor_msgs::LaserScan::ConstPtr& scaninfo);
  
  //Related with the mode chose
  void mode_choice();
  void manualDriving();
  void autonomousDriving();
  
  bool check_timeout();
  
  //ACTIONCLIENT RELATED
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& fb);
  void doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCb();


  
  //SERVICES RELATED
  bool switch_mode(final_assignment::Behavior_mode_service::Request  &req, final_assignment::Behavior_mode_service::Response &res);
  bool set_goal(final_assignment::Goal_service::Request &req, final_assignment::Goal_service::Response &res);
 // bool check_timeout(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
  // ROS NodeHandle
  ros::NodeHandle node_handle;
  

  //ACTION CLIENT
 ActionClient ac;

  //PUBLISHER
  ros::Publisher pubStateInfo;
  ros::Publisher pubCmdVel;
  ros::Publisher pubTimeout;
   		
  // SUBSCRIBERS
  ros::Subscriber subMode;
  ros::Subscriber subCmdVelRemapped;
  ros::Subscriber subScanner;
 
  
  //SERVICES
  ros::ServiceServer service_mode;
  ros::ServiceServer service_goal; 
  //ros::ServiceServer service_timeout; 
  
  int current_mode;
  bool goal_is_defined; 
  bool isActionActive;
  double x_goal;
  double y_goal;
  double currentpose_x;
  double currentpose_y;
  bool isTimeout;
  bool isArrived;
  bool manual;
  bool assisted;
  
  //std::string GoalID;
  geometry_msgs::Twist velFromTeleop; // Velocity sent over from teleop_twist_keyboard

  
  
};

#endif 
