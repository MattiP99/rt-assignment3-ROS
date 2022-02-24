#ifndef USER_H
#define USER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"
#include <std_srvs/SetBool.h>

class UserClass{
	public:
 	
 		UserClass(ros::NodeHandle* nodehandle);
  		~UserClass();
  		
  		//FUNCTIONS
  		
  		void timeoutTimerCallback(const ros::TimerEvent& event);
  		int mode_choice();
  		int getUserChoice();
  		void cancelGoal();
  		void receiveStateInfo(const std_msgs::Bool::ConstPtr& info); 
  		

	private:
  		// ROS NodeHandle
  		ros::NodeHandle node_handle;

  		// reate Assync spiner
  		ros::AsyncSpinner spinner;
  		ros::CallbackQueue Queue;
  		
  		//SUBSCRIBER
  		ros::Subscriber subStateInfo;
  		
  		
		//PUBLISHERS
  		
  		ros::Publisher pub_mode;
  		ros::Publisher pub_cancel;
  		
  		//CLIENTS
  		
   		ros::ServiceClient client_mode;
   		ros::ServiceClient client_goal;
   		ros::ServiceClient client_timeout;
  
  		bool isComplete;
 		bool isAutonomous;
 		bool isUserDeciding;
 		bool isTimeout;
 		
 		double inputX;
 		double inputY;
 		int inputChoice;
 		int userInput;
 		
 		
  		
};

#endif 


  	
  	
