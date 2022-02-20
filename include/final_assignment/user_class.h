#ifndef USER_H
#define USER_H

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/string.h"
#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"

class UserClass{
	public:
 	
 		UserClass();
  		~UserClass();
  		
  		//FUNCTIONS
  		void sendInfo(std_msgs::string msg);
  		void timeoutTimerCallback(const ros::TimerEvent& event)
  		void mode_choice();
  		int getUserChoice();
  		int cancelGoal();
  		

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
  
  		bool isComplete;
 		bool isAutonomous;
 		bool isUserDeciding;
 		double inputX;
 		double inputY;
 		int inputChoice;
 		int userInput;
 		
  		
};

#endif 


  	
  	
