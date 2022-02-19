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
#include <geometry_msgs/Point.h>
#include "geometry_msgs/Twist.h"


//services needed
#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

const int TEXT_DELAY = 25000; // microseconds
final_assignment::Goal_service goal_srv;
final_assignment::Behavior_mode_service mode_srv;

UserClass::UserClass() : node_handle(""), spinner(0) {

  ROS_INFO("Init Started");

  spinner.start();
  
  
  isUserDeciding = true;
  isComplete = false;
  isAutonomous = false;
  
  input_x = 0;
  input_y = 0;
  //PUBLISHERS
  pubStateInfo = node_handle.advertise<std_msgs::String>("controller_stateinfo", 10);
  pub_cancel = node_handle.advertise<std_msgs::String>("/cancel", 1);
  pub_mode = node_handle.advertise<std_msgs::Int32>("/current_mode",1);
  
  //SERVICES
  
  client_mode = nh.serviceClient<second_assignment::Behavior_mode_service>("/switch_mode");
  client_goal = nh.serviceClient<final_assignment::Goal_service>("/set_goal");
  
   
  
  
  ROS_INFO("Init Finished");
}

UserClass::~UserClass() { ros::shutdown(); }


void UserClass::sendInfo(std::string msg){
  	std_msgs::String stateInfoMsg;
  	stateInfoMsg.data = msg;
  	this.pubStateInfo.publish(stateInfoMsg);
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
      
        double actionTimeout;
	//USING SERVICE
	mode_srv.request.mode = 1;
	
	//CLASS VARIABLES
        isUserDeciding = false;
        isComplete = false;
        isAutonomous = true;
        
        if (node_handle.getParam("/rt1a3_action_timeout", actionTimeout)) {
          // ROS_INFO("Action timeout successfully retrieved from parameter server");
        } else {
          ROS_ERROR("Failed to retrieve action timeout from parameter server");
        }

        clearTerminal();

        displayText("Mode: ", TEXT_DELAY);
        terminalColor(36, true);
        displayText("Autonomous Goal Point\n", TEXT_DELAY);
        terminalColor(37, true);
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
	client_goal.waitForExistence();
	client_goal.call(goal_srv); //MAYBE CHECK SOMETHING
	
  

        clearInputBuffer();
        clearTerminal();
        terminalColor(37, false);
        displayText("Driving...\n", TEXT_DELAY);
        terminalColor(36, false);
        displayText("\nPress q to cancel the goal, or any other key to continue.\n", TEXT_DELAY);
	
	
        // Start counter to timeout
        ros::Timer timeoutTimer = node_handle.createTimer(ros::Duration(actionTimeout), timeoutTimerCallback);
        // User is allowed to cancel the robot's goal point: listen for input!
        // The input must be asynchronous!
	std::cin >> s;
        

        while (true) {
          if (isComplete) {
            displayText("\nTerminating program...\n", TEXT_DELAY);
            ros::Duration(2).sleep();
            return 0; // Exit because we are done!
          }
          else{
          	//if(s.c_str()=="q"){
			this.cancelGoal();          		
          	}
          }
	      
        //ros::spinOnce();
        ros::Duration(1).sleep();
        break;
      

      case 2: // Choice 2: Manual Driving
      //USING SERVICE
	
	mode_srv.request.mode = 2;
        isUserDeciding = false;

        clearTerminal();

        displayText("Mode: ", TEXT_DELAY);
        terminalColor(36, true);
        displayText("Manual Drive\n", TEXT_DELAY);

        //pubManualDrive.publish(msgBool);

        break;

      case 3: // Choice 3: Assisted Driving
      //USING SERVICE
	mode_srv.request.mode = 3;
	
	//CLASS VARIABLES
        isUserDeciding = false;

        clearTerminal();

        displayText("Mode: ", TEXT_DELAY);
        terminalColor(36, true);
        displayText("Assisted Drive\n", TEXT_DELAY);

        //pubAssistedDrive.publish(msgBool);

        break;

      default:
        terminalColor(41, true);
        displayText("\nInvalid. Please select a valid option.\n", TEXT_DELAY);
        terminalColor(37, false);
        ros::Duration(1).sleep();
        inputChoice = getUserChoice();
        break;
    }
    
    client_mode.waitForExistence(); //MAYBE IN THE WRONG POSITION
    client_mode.call(mode_srv);
  }
 
  // NEW PART
  // ros::waitForShutdown();
  //ros::spin();

  return 0;
}


int cancelGoal () {
  std::string inputStr;
  
  actionlib_msgs::GoalID goalCancelID;
  terminalColor(37, false);
  ROS_INFO("\n Press q in order to cancel the goal or anyother key to continue")
  std::cin >> inputStr;

  if (!isComplete) {
    if (inputStr.c_str() == "q") {
      // "q" pressed - cancel goal!
      clearTerminal();
      terminalColor(37, false);
      ROS_INFO("Autonomous driving cancel request by user.");
      displayText("Sending goal cancel request...\n", TEXT_DELAY);

      // Setting fields for cancel request
      goalCancelID.stamp.sec = 0;
      goalCancelID.stamp.nsec = 0;
      goalCancelID.id = "";

      pub_cancel.publish("cancel");
      terminalColor(32, false);
      displayText("\nGoal has been cancelled.\n", TEXT_DELAY);

      isComplete = true;
    } else {
      clearTerminal();
      terminalColor(37, false);
      displayText("Driving...\n", TEXT_DELAY);
      terminalColor(36, false);
      displayText("\nPress q to cancel the goal, or any other key to continue.\n", TEXT_DELAY);
    }
  }
}



int UserClass::getUserChoice () {
  std::string s;
  clearTerminal();
  terminalColor(37, false);
  displayText("Choose one of the following options: \n", TEXT_DELAY);
  terminalColor(32, false);
  displayText("1. Autonomously reach coordinates\n", TEXT_DELAY);
  displayText("2. Manual driving\n", TEXT_DELAY);
  displayText("3. Assisted driving\n", TEXT_DELAY);
  terminalColor(37, false);
  std::cin >> s;
  inputChoice = atoi(s.c_str());
  

  return inputChoice;
}
  	
    


   
   void UserClass::timeoutTimerCallback(const ros::TimerEvent& event) {
  	isComplete = true;

  	clearTerminal();
  	terminalColor(31, true);
  	std::cout << "Action timed out! (Are you sure the goal was reachable?)\n";
  	fflush(stdout);
	}


int main (int argc, char **argv)
{
 	ros::init(argc, argv, "user_node");
 	ros::NodeHandle nh;
   	
  	UserClass us = UserClass(&nh);
  	
    	ros::waitForShutdown();
}




