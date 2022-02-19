#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <"std_msgs/String.h">
#include <"final_assignment/Behavior_mode_service.h>
#include <"fianl_assignment/Goal_service.h">

class UserClass {
	public:
 		bool isComplete;
 		bool isAutonomous;
 		double inputX;
 		double inputY;
 		int inputChoice;
 		int userInput;
 		bool isUserDeciding;
 		
 		
 		UserClass();
  		virtual ~UserClass();

	private:
  		// ROS NodeHandle
  		ros::NodeHandle node_handle;

  		// reate Assync spiner
  		ros::AsyncSpinner spinner;
  
  		

  
  		//PUBLISHERS
  		ros::Publisher pubStateInfo;
  		ros::Publisher pub_mode;
  		ros::Publisher pub_cancel;
  		
  		//CLIENTS
  		
   		ros::ServiceClient client_srv;
   		ros::ServiceClient client_res;
  
  		//FUNCTIONS
  		void sendInfo(std_msgs::string msg);
  		void timeoutTimerCallback(const ros::TimerEvent& event)
  		void mode_choice();
  		int getUserChoice();
  		int cancelGoal();
  		bool switch_mode(final_assignment::Behavior_mode_service::Request  &req, final_assignment::Behavior_mode_service::Response &res);
  		bool set_goal(final_assignment::Goal_service::Request  &req, final_assignment::Goal_service::Goal::Response &res);
};

#endif 


  	
  	
