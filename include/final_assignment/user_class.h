#ifndef USER_H
#define USER_H

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"

class UserClass{
	public:
 	
 		UserClass(ros::NodeHandle* nodehandle);
  		~UserClass();
  		
  		//FUNCTIONS
  		
  		void timeoutTimerCallback(const ros::TimerEvent& event);
  		int mode_choice();
  		int getUserChoice();
  		void cancelGoal();
  		void receiveStateInfo(const std_msgs::String::ConstPtr& info); 
  		

	private:
  		// ROS NodeHandle
  		ros::NodeHandle node_handle;

  		// reate Assync spiner
  		ros::AsyncSpinner spinner;
  		
  		//SUBSCRIBER
  		ros::Subscriber subStateInfo;
  		
		//PUBLISHERS
  		
  		ros::Publisher pub_mode;
  		ros::Publisher pub_cancel;
  		
  		//CLIENTS
  		
   		ros::ServiceClient client_mode;
   		ros::ServiceClient client_goal;
  
  		bool isComplete;
 		bool isAutonomous;
 		bool isUserDeciding;
 		double inputX;
 		double inputY;
 		int inputChoice;
 		int userInput;
 		
  		
};

#endif 


  	
  	
