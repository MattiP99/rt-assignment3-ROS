#include "ros/ros.h"
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <cstdlib>
#include <string>
#include <math.h>
#include "final_assignment/utils.h"
#include "final_assignment/controller_class.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include "final_assignment/Behavior_mode_service.h"
#include "final_assignment/Goal_service.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <std_srvs/SetBool.h>
#include <ros/callback_queue.h>


const int TEXT_DELAY = 25000; // microseconds
double actionTimeout;
double brakethreshold;
//ros::Timer timeoutTimer;
//ACTIONCLIENT
  
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;
ActionClient *acPointer;
move_base_msgs::MoveBaseGoal goal;
std::chrono::high_resolution_clock::time_point time_start;  
std::chrono::high_resolution_clock::time_point time_end;



// Represents MINIMUM distances of obstacles around the robot
struct minDistances {
  float left;
  float right;
};

static struct minDistances minDistances;

//These values set the ros parameters but actually the ros param server values are used
const double ACTION_TIMEOUT_DEFAULT = 30.0; //seconds
const double BRAKE_THRESHOLD_DEFAULT = 1.0;
const double GOAL_TH = 0.5;





//CONSTRUCTOR
ControllerClass::ControllerClass(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle2,ros::NodeHandle* nodehandle3) : node_handle(*nodehandle), node_handle2(*nodehandle2), node_handle3(*nodehandle3),spinner2(1,&secondQueue), spinner3(1,&thirdQueue){ // node_handle2(""),node_handle3(""), spinner2(0,&secondQueue), spinner3(0,&thirdQueue) {
  
  x_goal = 0;
  y_goal = 0;
  goal_is_defined = false;
  isTimeout = false;
  isArrived = false;
  manual = false;
  assisted = false;
  ROS_INFO("Init Started");

  //spinner.start();
  
  
  pubStateInfo = node_handle.advertise<std_msgs::Bool>("controller_stateinfo", 10);
  pubCmdVel = node_handle.advertise<geometry_msgs::Twist>("cmd_vel",10);
  
  
  
  ros::SubscribeOptions ops;
  ops.template initByFullCallbackType<std_msgs::String>("/cancel",1, boost::bind(&ControllerClass::CancelCallBack, this, _1));
  ops.allow_concurrent_callbacks = true;
  
  subMode = node_handle.subscribe(ops);
   //= node_handle.subscribe("/current_mode",1, &ControllerClass::ModeCallBack, this);
  
  // Create a second NodeHandle
  
  //node_handle2.setCallBackQueue(&secondQueue);
  ros::CallbackQueue secondQueue;
  node_handle2.setCallbackQueue(&secondQueue);
  secondQueue.callAvailable(ros::WallDuration());
  subCmdVelRemapped = node_handle2.subscribe("controller_cmd_vel", 10, &ControllerClass::UserDriveCallBack,this);
  
  // Spawn a new thread for high-priority callbacks.
  //std::thread prioritySpinThread([&secondQueue]() {
  //ros::SingleThreadedSpinner spinner;
  //spinner.spin(&secondQueue);
  //});
  //prioritySpinThread.join();
  
  // Create a third NodeHandle
  ros::CallbackQueue thirdQueue;
  node_handle3.setCallbackQueue(&thirdQueue);
  thirdQueue.callAvailable(ros::WallDuration());
  subScanner = node_handle3.subscribe("scan", 1, &ControllerClass::LaserScanParserCallBack, this);
 
  
  service_mode = node_handle.advertiseService("/switch_mode", &ControllerClass::switch_mode, this);
  service_goal = node_handle2.advertiseService("/set_goal", &ControllerClass::set_goal, this);
  service_timeout = node_handle3.advertiseService("/timeout", &ControllerClass::check_timeout, this);
  
  spinner2.start();
  spinner3.start();
  
  ROS_INFO("Init Finished");
  
  mode_choice();
  ros::waitForShutdown();
}

//DISTRUCTOR
ControllerClass::~ControllerClass() { ros::shutdown(); }


// service for checking the timeout and telling the user
bool ControllerClass::check_timeout(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
	if(isTimeout == true){
		res.success = true;
	}
	else{
		res.success = false;
	}
	return true;

}


//service to set goal
bool ControllerClass::set_goal(final_assignment::Goal_service::Request  &req, final_assignment::Goal_service::Response &res){

	if(req.x!= 0 && req.y!= 0){
		x_goal= req.x;
		y_goal = req.y;
		goal_is_defined = true;
		time_start = std::chrono::high_resolution_clock::now();
		
		res.success = true;
	}
	else{
		res.success = false;
	}
	return true;
}


//service to change mode
bool ControllerClass::switch_mode(final_assignment::Behavior_mode_service::Request& req, final_assignment::Behavior_mode_service::Response& res){
   
  
    //The request is of type int32 (by definition of the service), there is no need to check the type
    
    if (req.mode ==2 or req.mode == 3){
        current_mode = req.mode;
        res.success = true;
        ROS_INFO("The current mode is now %d ", current_mode);
        }
       //An invalid interger will no affect the system
    else{
        res.success = false;
        displayText("Please enter an existing mode, i.e. 1, 2 or 3", TEXT_DELAY);
    }
    
       return true;
   }
   
void ControllerClass::printTeleop(){
//Different message according to the mode
        if (current_mode ==2 or current_mode==3){
        manual = true;
        assisted = false;
            clearTerminal();
            
            displayText("Mode: ", TEXT_DELAY);
            terminalColor('3');
            displayText("Manual Drive\n", TEXT_DELAY);
            terminalColor('3');
            displayText("\nKeys to control the robot:\n"
                    "---------------------------\n"
                    "Moving around:\n"
                    "u    i    o\n"
                    "j    k    l\n"
                    "m    ,    .\n"

                    "q/z : increase/decrease max speeds by 10 percent\n"
                    "w/x : increase/decrease only linear speed by 10 percent\n"
                    "e/c : increase/decrease only angular speed by 10 percent\n"
                    "anything else : stop\n\n\n", TEXT_DELAY);
        }else{
            printf("\nTo set a goal, call the service /set_goal\n");
        }

} 
void ControllerClass::manualDriving(){
	std::string input_assisted;
	printTeleop();
	manual = true;
        assisted = false;
	while(ros::ok()){
		displayText("if you want to enable the assisted driving press a or p if you want to exit", TEXT_DELAY);
		std::cin >> input_assisted;
		if(input_assisted == "a"){
			manual = false;
			displayText("assisted driving mode enabled", TEXT_DELAY);
			assisted = true;
			collisionAvoidance();
		}else if (input_assisted == "p"){
			displayText("closing ros", TEXT_DELAY);
			ros::shutdown();
		}
	
	}
 }
 
	
void ControllerClass::autonomousDriving(){

     ActionClient ac("move_base", true);
     acPointer = &ac;
     move_base_msgs::MoveBaseActionFeedback status;
     
    if (node_handle.getParam("/rt1a3_action_timeout", actionTimeout)) {
     ROS_INFO("Action timeout successfully retrieved from parameter server");
     } else {
    	ROS_ERROR("Failed to retrieve action timeout from parameter server");
  	}
    
    //The goal is only to be considered if the mode 1 is active
    //timeoutTimer = node_handle.createTimer(ros::Duration(actionTimeout), &ControllerClass::timeoutTimerCallback,this); //NON SONO SICURI DI COME SI SCRIVA LA FUNZIONE NEL TIMER!!!!!!
        
        // Set the starting time
	
	if(goal_is_defined){
		//update the new goal, regarless where this goal is in the map
        	//ROS_INFO("request is x=%f and y=%f", req.x, req.y);
        	goal.target_pose.header.frame_id = "map";
        	goal.target_pose.pose.orientation.w = 1.;
        	goal.target_pose.pose.position.x = x_goal;
        	goal.target_pose.pose.position.y = y_goal;
        
        	ROS_INFO("The goal is now set to (%f, %f): ", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        	//ac.waitForServer();
        	ac.sendGoal(goal);
        	
        	
    		// If there is a goal check if the robot has reached the goal or is trying to reach the goal for a too long time
    		if(goal_is_defined)
    		{
    		/*
    			// Take the current robot position
        		double dist_x;
			double dist_y;
			double lin_dist;
			std::string linear;
    	
			// Compute the error from the actual position and the goal position
			dist_x = currentpose_x - x_goal;
			dist_y = currentpose_y - y_goal;
			lin_dist = sqrt(dist_x*dist_x + dist_y*dist_y);
			
			linear = std::to_string(lin_dist);
			
			
			displayText("the linaer distance from the goal is\n" ,TEXT_DELAY);
			displayText(linear,TEXT_DELAY);
	
			// The robot is on the goal position
			if (abs(dist_x) <= GOAL_TH && abs(dist_y) <= GOAL_TH)
			{
			     printf("Goal reached\n");
			     isArrived = true;
			     ac.cancelGoal();
			     //pubblica il fatto che sia arrivato per l'user       //NON SO SE è GIUSTO COSI RiGUARDA
			     ros::shutdown();
			}

	    		//time_end = std::chrono::high_resolution_clock::now();
	        	//auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
   			time_end = std::chrono::high_resolution_clock::now();
        		auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
        	
        		if(time > actionTimeout) //TIMEOUT)
        			{
        			isTimeout = true;
        			printf("The actual goal can't be reached!\n");
        			ac.cancelGoal(); // Cancel the goal if it can't be reached
        			ros::shutdown();
        			//EXIT? ROS_SHUTDOWN?
       		 }
    		*/
        	
       	ac.waitForServer();
  		if (ac.waitForResult(ros::Duration(actionTimeout))) {
    			actionlib::SimpleClientGoalState state = ac.getState();
   			 ROS_INFO("Action finished: %s", state.toString().c_str());
   			 
   			 // If action does not finish before timeout, then cancel it and notify user
    			 sendInfo(true);
    			 ac.cancelGoal();
 		 }else {
    			ROS_INFO("Action timed out.");
    			sendInfo(false);
    			ac.cancelGoal();
    		}
    		}
	}
}

void ControllerClass::mode_choice(){
	
	if(!node_handle.hasParam("rt1a3_action_timeout")){
  		node_handle.setParam("/rt1a3_action_timeout", ACTION_TIMEOUT_DEFAULT);
  	}
  	
 	 //Set the brake threashold on the parameter server so it can be tweaked in runtime
 	 if(!node_handle.hasParam("rt1a3_brake_threshold")){
 	 	node_handle.setParam("/rt1a3_brake_threshold", BRAKE_THRESHOLD_DEFAULT);
  	}
  	
  	
	if (current_mode == 1){
	    autonomousDriving();
	    
	}else if(current_mode == 2){
	    manualDriving();
	
	}else if(current_mode == 3){
	    collisionAvoidance();
	
	}
	else{
	    displayText("mode not set properly", TEXT_DELAY);
	}
	 
}

void ControllerClass::LaserScanParserCallBack(const sensor_msgs::LaserScan::ConstPtr& scaninfo) {
 	const int NUM_SECTORS = 2;
 	int numElements;
 	int numElementsSector;
 	float leftDistMin;
 	float rightDistMin;
	
        if(assisted = true ){
        numElements = scaninfo->ranges.size();
  	numElementsSector = numElements/NUM_SECTORS;
  	// Temporarily take an element from each range
  	leftDistMin = scaninfo->ranges[0];
  	rightDistMin = scaninfo->ranges[numElements - 1];

  	for (int i = 0; i < numElements; i++) {
   		if (i < numElementsSector) {
      		// FIRST sector
      			if (scaninfo->ranges[i] < leftDistMin) {
        			leftDistMin = scaninfo->ranges[i];
      			}
    		} else {
      		// THIRD sector
      			if (scaninfo->ranges[i] < rightDistMin) {
        			rightDistMin = scaninfo->ranges[i];
      			}
    		}
  	}

  	minDistances.left = leftDistMin;
  	minDistances.right = rightDistMin;
        
        }
  	
}

void ControllerClass::collisionAvoidance() {
	
  	geometry_msgs::Twist newVel;
  	double brakeThreshold;

  	newVel = velFromTeleop;

  	if (node_handle.getParam("/rt1a3_brake_threshold", brakeThreshold)) {
    		// ROS_INFO("Brake threshold successfully retrieved from parameter server");
  	} else {
    		ROS_ERROR("Failed to retrieve brake threshold from parameter server");
  	}

  	// Correct user input
  	if (minDistances.left <= brakeThreshold) {
    		newVel.linear.x = velFromTeleop.linear.x/2;
    		newVel.angular.z = 1; // Turn the other way
    		sendInfo("Obstacle detected! Collision avoidance in progress.");
  	} else if (minDistances.right <= brakeThreshold) {
    		newVel.linear.x = velFromTeleop.linear.x/2;
    		newVel.angular.z = -1; // Turn the other way
    		sendInfo("Obstacle detected! Collision avoidance in progress.");
  	} else {
    		sendInfo("Listening to commands.");
  	}

  	pubCmdVel.publish(newVel);
}


void ControllerClass::UserDriveCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
  	velFromTeleop = *msg; // Save new velocity as class variable
}


void initActionClients()
{
  ActionClient ac("move_base", true);
  acPointer = &ac;
  ac.waitForServer();
  
}

void ControllerClass::sendInfo(bool temp){
	if(temp == true){
		//Action finished before the timeout
		isArrived = true;
	}else{
		//time Expired
		
	
	}
  	std_msgs::Bool msg;
  	msg.data = temp;
  	pubStateInfo.publish(msg);
  }	
	
void ControllerClass::currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& status) {
     ActionClient ac("move_base", true);
     acPointer = &ac;
    
    // Update the goal ID if there is a new goal
    if (GoalID != status->status.goal_id.id) {
        GoalID = status->status.goal_id.id;
    }
    
     currentpose_x = status->feedback.base_position.pose.position.x;
     currentpose_y = status->feedback.base_position.pose.position.y;
   
    }


void ControllerClass::CancelCallBack(const std_msgs::String &msg){
    ActionClient ac("move_base", true);
    acPointer = &ac;
    if( msg.data == "cancel"){
    	
    	ac.cancelGoal();
    	clearTerminal();
    	terminalColor('4');
    	displayText("The goal has been canceled", TEXT_DELAY);
    	ros::shutdown();
    }
    
}

int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "final_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  ros::NodeHandle nh3;
  initActionClients();
  
  
  
  ControllerClass controller_object(&nh,&nh2,&nh3);
  

  ros::waitForShutdown();

  return 0;
}









