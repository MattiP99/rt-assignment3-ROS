#include "ros/ros.h"
#include <unistd.h>
#include <termios.h>


//classes and functions needed
#include "final_assignment/utils.h"
#include "final_assignment/user_class.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

//messages needed
#include "std_msgs/Int32.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/Twist.h"


//services needed
#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/callback_queue.h>

const int TEXT_DELAY = 25000; // microseconds
final_assignment::Goal_service goal_srv;
final_assignment::Behavior_mode_service mode_srv;


UserClass::UserClass(ros::NodeHandle* nodehandle): node_handle(*nodehandle), spinner(1,&Queue) {

  ROS_INFO("Init Started");
  isUserDeciding = true;
  isComplete = false;
  isAutonomous = false;
  isTimeout = false;
  
  inputX = 0;
  inputY = 0;
  
  //SUBSCRIBER
  ros::CallbackQueue Queue;
  node_handle.setCallbackQueue(&Queue);
  Queue.callAvailable(ros::WallDuration());
  subStateInfo =node_handle.subscribe("controller_stateinfo", 100, &UserClass::receiveStateInfo, this);
  spinner.start();
  
  //PUBLISHERS
  
  pub_cancel = node_handle.advertise<std_msgs::String>("/cancel", 1);
  pub_mode = node_handle.advertise<std_msgs::Int32>("/current_mode",1);
  
  //SERVICES
  
  client_mode = node_handle.serviceClient<final_assignment::Behavior_mode_service>("/switch_mode");
  client_goal = node_handle.serviceClient<final_assignment::Goal_service>("/set_goal");
  client_timeout = node_handle.serviceClient<std_srvs::SetBool>("/timeout");
  spinner.start();
   
  
  
  ROS_INFO("Init Finished");
  mode_choice();
  
}

UserClass::~UserClass() { ros::shutdown();}



void UserClass::receiveStateInfo(const std_msgs::Bool::ConstPtr& info){

 isComplete = info->data;
 if(isComplete == true){
 	clearTerminal();
 	
 	displayText("the goal has been reached\n ", TEXT_DELAY);
        terminalColor('3');
        displayText("closing ROS\n", TEXT_DELAY);
        terminalColor('9');
        ros::shutdown();
 	}
 
} 

int UserClass::mode_choice(){
  
  std::string s;
  // Get user input on what to do. Then, publish the appropriate information to
  // the controller node.
  ROS_INFO("Waiting for user input");
  while (isUserDeciding) {
    inputChoice = getUserChoice();
    

    switch(inputChoice) {
      case 1: // Choice 1: Autonomous Goal Point
      {
        double actionTimeout;
	//USING SERVICE
	mode_srv.request.mode = 1;
	
	//CLASS VARIABLES
        //isUserDeciding = false;
        isComplete = false;
        isAutonomous = true;
        
        if (node_handle.getParam("/rt1a3_action_timeout", actionTimeout)) {
           ROS_INFO("Action timeout successfully retrieved from parameter server");
        } else {
          ROS_ERROR("Failed to retrieve action timeout from parameter server");
        }

        clearTerminal();

        displayText("Mode: ", TEXT_DELAY);
        terminalColor('3');
        displayText("Autonomous Goal Point\n", TEXT_DELAY);
        terminalColor('9');
        isUserDeciding = true;
        
        // Get user input
        while (isUserDeciding) {
          std::string inputX_str;
          std::string inputY_str;
          displayText("Input goal point coordinates\n", TEXT_DELAY);
          displayText("X: ", TEXT_DELAY);
          std::cin >> inputX_str;
          displayText("Y: ", TEXT_DELAY);
          std::cin >> inputY_str;

          // Check if input is numeric or not
          if (!isStringNumeric(inputX_str) || !isStringNumeric(inputY_str)) {
            displayText("Invalid input. Please insert numeric coordinates!\n", TEXT_DELAY);
          } else {
            // Good input!
            isUserDeciding = false;
            inputX = atof(inputX_str.c_str());
            inputY = atof(inputY_str.c_str());
          }
        }
	
	goal_srv.request.x = inputX;
	goal_srv.request.y = inputY;
	//MAYBE CHECK SOMETHING
	clearInputBuffer();
        clearTerminal();
        terminalColor('3');
        displayText("Driving...\n", TEXT_DELAY);
        terminalColor('9');
        
      }  break;
      

      case 2: // Choice 2: Manual Driving
      {
      //USING SERVICE
	
	mode_srv.request.mode = 2;
        isUserDeciding = false;

        clearTerminal();

        displayText("Mode: ", TEXT_DELAY);
        terminalColor('3');
        displayText("Manual Drive\n", TEXT_DELAY);

        //pubManualDrive.publish(msgBool);

       } break;
	
      case 3: // Choice 3: Assisted Driving
      //USING SERVICE
      {
	mode_srv.request.mode = 3;
	
	//CLASS VARIABLES
        isUserDeciding = false;

        clearTerminal();

        displayText("Mode: ", TEXT_DELAY);
        terminalColor('3');
        displayText("Assisted Drive\n", TEXT_DELAY);

        //pubAssistedDrive.publish(msgBool);

        }break;
	
      default:
      {
        terminalColor('3');
        displayText("\nInvalid. Please select a valid option.\n", TEXT_DELAY);
        terminalColor('9');
        ros::Duration(1).sleep();
        inputChoice = getUserChoice();
        }break;
        
    	}
    }
    client_mode.waitForExistence(); //MAYBE IN THE WRONG POSITION
    if(client_mode.call(mode_srv)){
    	if(mode_srv.response.success == true){
    		displayText("success in calling mode server",TEXT_DELAY);
    		}else{
    		displayText("error in contacting the server",TEXT_DELAY);
    		}
    	}
    
    client_goal.waitForExistence();
    if(client_goal.call(goal_srv)){
    		  if(goal_srv.response.success == true){
			displayText("INPUT RECEIVED CORRECTLY, starting the timer",TEXT_DELAY);
			cancelGoal();
		 }else
			displayText("input not yet received",TEXT_DELAY);
		
	}
    			
    
  // NEW PART
  // ros::waitForShutdown();
  //ros::spin();

  return 0;
}


void UserClass::cancelGoal () {
  std::string inputStr;
  std::string cancel;
  std_srvs::SetBool timeout_srv;
   
 
  std_msgs::String cancel_msg;
  cancel_msg.data = cancel;
  
  while(!isComplete){
  
     if(client_timeout.call(timeout_srv)){
     	isTimeout = timeout_srv.response.success;
  	
  	if( isTimeout == true) {
  		displayText("time is ran out, target non reachable",TEXT_DELAY);
  		ros::shutdown();
  	}
  	else{
		terminalColor('3');
  		ROS_INFO("\n Press q in order to cancel the goal or anyother key to continue");
  		std::cin >> inputStr;
    		
    		if (inputStr.c_str() == "q") {
      			// "q" pressed - cancel goal!
      			clearTerminal();
      			terminalColor('3');
      			displayText("Autonomous driving cancel request by user.",TEXT_DELAY);
      			displayText("Sending goal cancel request...\n", TEXT_DELAY);

            		cancel= "cancel";
      			cancel_msg.data = cancel.c_str();

      			pub_cancel.publish(cancel_msg);
      			
      			

      			
    			} 
		  	
	  }
     }
  }
}
  



int UserClass::getUserChoice () {
  std::string s;
  clearTerminal();
  terminalColor('3');
  displayText("Choose one of the following options: \n", TEXT_DELAY);
  terminalColor('9');
  displayText("1. Autonomously reach coordinates\n", TEXT_DELAY);
  displayText("2. Manual driving\n", TEXT_DELAY);
  displayText("3. Assisted driving\n", TEXT_DELAY);
  terminalColor('3');
  std::cin >> s;
  inputChoice = atoi(s.c_str());
  

  return inputChoice;
}
  	
   


int main (int argc, char **argv)
{
 	ros::init(argc, argv, "final_ui");
 	ros::NodeHandle nh;
   	
  	UserClass us(&nh);
  	
    	ros::waitForShutdown();
}




