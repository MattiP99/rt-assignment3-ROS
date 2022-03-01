

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
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <boost/bind.hpp>

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

move_base_msgs::MoveBaseGoal goal;
//std::chrono::high_resolution_clock::time_point time_start;  
//std::chrono::high_resolution_clock::time_point time_end;



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
ControllerClass::ControllerClass(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle2) : node_handle(*nodehandle),node_handle2(*nodehandle2), ac("move_base",true){
  
  x_goal = 0;
  y_goal = 0;
  goal_is_defined = false;
  isTimeout = false;
  isArrived = false;
  manual = false;
  assisted = false;
  isActionActive = false;
  ROS_INFO("Init Started");
  
  
  pubStateInfo = node_handle.advertise<std_msgs::Bool>("controller_stateinfo", 100);
  pubCmdVel = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",100);
  pubTimeout = node_handle.advertise<std_msgs::Bool>("/timeout",100);
   
  
  // Create a second NodeHandle
  subCmdVelRemapped = node_handle.subscribe("/controller_cmd_vel", 100, &ControllerClass::UserDriveCallBack,this);
 
  
  // Create a third NodeHandle
  
  subScanner = node_handle.subscribe("scan", 10, &ControllerClass::LaserScanParserCallBack, this);
 
  ros::SubscribeOptions ops;
  ops.template initByFullCallbackType<std_msgs::String>("/cancel",1, boost::bind(&ControllerClass::CancelCallBack, this, _1));
  ops.allow_concurrent_callbacks = true;
  
  
  subMode = node_handle2.subscribe(ops);
  service_mode = node_handle.advertiseService("/switch_mode", &ControllerClass::switch_mode, this);
  service_goal = node_handle.advertiseService("/set_goal", &ControllerClass::set_goal, this);
  
  
  ROS_INFO("Init Finished");
  ros::Duration(10).sleep();
 
  
  
}

//DISTRUCTOR
ControllerClass::~ControllerClass() { ros::shutdown(); }


//Call back of the sendGoal for when the goal is reached
void ControllerClass::doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result)
  {
   
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    if( state == actionlib::SimpleClientGoalState::SUCCEEDED){
    	isArrived = true;
    	isActionActive = false;
    }
    
  }


void ControllerClass::activeCb()
{
  actionlib::SimpleClientGoalState status = ac.getState();
  //ros::Rate loop_rate(0.1);
  if(status==actionlib::SimpleClientGoalState::ACTIVE or status==  actionlib::SimpleClientGoalState::PENDING)
   {
   	displayText("Goal just went active\n",TEXT_DELAY);
   	isActionActive = true;
   } else {
   	displayText("\nAction is not more active\n",TEXT_DELAY);
   	isActionActive = false;
   }
  }
  
  

//Function for when the robot is approching the goal
void ControllerClass::feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr&  msg) {
 
     currentpose_x = msg->base_position.pose.position.x;
     currentpose_y = msg->base_position.pose.position.y;
     
     displayText("\n\nX:  ",TEXT_DELAY);
     displayText(std::to_string(currentpose_x),TEXT_DELAY);
     displayText(" and  ",TEXT_DELAY);
     displayText("Y:  ",TEXT_DELAY);
     displayText(std::to_string(currentpose_y),TEXT_DELAY);
    
     	
     
   
    }

//For checking the timer and telling the user if it is expired or not by publishing a topic
bool ControllerClass::check_timeout(){
        std_msgs::Bool time;
	
	if(isTimeout == true){
		time.data = true;
		pubTimeout.publish(time);
	}
	else {
	if (isArrived == true){
		time.data = false;
		pubTimeout.publish(time);
	}
	}
	return true;

}


//service to set goal
bool ControllerClass::set_goal(final_assignment::Goal_service::Request  &req, final_assignment::Goal_service::Response &res){
	
	std::string req_strX;
	std::string req_strY;
	
	req_strX = std::to_string(req.x);
	req_strY = std::to_string(req.y);
	if(req_strX!= ""  && req_strY!= ""){
		displayText("\nGoal set properly", TEXT_DELAY);
		x_goal= req.x;
		y_goal = req.y;
		current_mode = 1;
		goal_is_defined = true;
		
		
		//time_start = std::chrono::high_resolution_clock::now();
		
		res.success = true;
		//autonomousDriving();
	}
	else{
		res.success = false;
	}
	return true;
}


//service to change mode
bool ControllerClass::switch_mode(final_assignment::Behavior_mode_service::Request& req, final_assignment::Behavior_mode_service::Response& res){
   
  
    //The request is of type int32 (by definition of the service), there is no need to check the type
    
    if (req.mode==1 or req.mode ==2 or req.mode == 3){
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
   

/*
* Manual drving
*/
void ControllerClass::manualDriving(){
	char input_assisted;
	
	manual = true;
        assisted = false;
	while(input_assisted!= 'p' or input_assisted!= 'a'){
		displayText("if you want to enable the assisted driving press a or p if you want to exit\n", TEXT_DELAY);
		std::cin >> input_assisted;
		if(input_assisted == 'a'){
			manual = false;
			displayText("assisted driving mode enabled", TEXT_DELAY);
			assisted = true;

			collisionAvoidance();
			break;
		}else if (input_assisted == 'p'){
			displayText("closing ros", TEXT_DELAY);
			ros::shutdown();
		}
	
	}
 }
 

/*
*function for handle the autonomous mode.
* After having set the parameter the action client set a goal with the user input
* and send this goal to the server in order to process it a starting the movement of the robot
*/	
void ControllerClass::autonomousDriving(){
     
     
     
     
    if (node_handle.getParam("/rt1a3_action_timeout", actionTimeout)) {
     std::cout << "\nAction timeout successfully retrieved from parameter server";
    fflush(stdout);
     } else {
    
     ROS_ERROR("Failed to retrieve action timeout from parameter server");
  	}
        
        // Set the starting time
	ac.waitForServer();
        	
        	
        	goal.target_pose.header.frame_id = "map";
        	goal.target_pose.pose.orientation.w = 1.;
        	goal.target_pose.pose.position.x = x_goal;
        	goal.target_pose.pose.position.y = y_goal;
        
        	std::cout <<"\nThe goal is now set to:  ";
        	fflush(stdout);
        	displayText(std::to_string(goal.target_pose.pose.position.x),TEXT_DELAY);
        	displayText(std::to_string(goal.target_pose.pose.position.y),TEXT_DELAY);
        	
        	ac.sendGoal(goal,
        		boost::bind(&ControllerClass::doneCb, this, _1, _2),
                	boost::bind(&ControllerClass::activeCb, this),
                	boost::bind(&ControllerClass::feedbackCb, this, _1)
                	);
                	
                
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
			
			
		displayText("\n\nthe linaer distance from the goal is   " ,TEXT_DELAY);
		displayText(linear,TEXT_DELAY);
	
		isTimeout = ac.waitForResult(ros::Duration(actionTimeout));
  		if (isTimeout) {
    			 actionlib::SimpleClientGoalState state = ac.getState();
   			 ROS_INFO("\nAction finished: %s", state.toString().c_str());
   			 
   			 // If action does not finish before timeout, then cancel it and notify user
   			 isActionActive = false;
   			 isArrived = true;
    			 sendInfo(true);
    			 ac.cancelGoal();
    			 
 		 }else {
    			ROS_INFO("\nAction timed out.");
    			isActionActive = false;
    			sendInfo(false);
    			ac.cancelGoal();
    			
    		}
   		//}
               	
                
    	}
                	
 
 /*
 * Callback for detecting obstacles from the laser on the board of teh robot
 */   
void ControllerClass::LaserScanParserCallBack(const sensor_msgs::LaserScan::ConstPtr& scaninfo) {
 	const int NUM_SECTORS = 2;
 	int numElements;
 	int numElementsSector;
 	float leftDistMin;
 	float rightDistMin;
	
        if(assisted == true){
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

/*
* Function called in assisted mode 
*/
void ControllerClass::collisionAvoidance() {
	manual = false;
	assisted = true;
  	geometry_msgs::Twist newVel;
  	double brakeThreshold;
  	char input_assisted;

  	newVel = velFromTeleop;
  	current_mode = 3;

  	if (node_handle.getParam("/rt1a3_brake_threshold", brakeThreshold)) {
    		// ROS_INFO("Brake threshold successfully retrieved from parameter server");
  	} else {
    		ROS_ERROR("Failed to retrieve brake threshold from parameter server");
  	}
  	
	

	while(input_assisted!= 'p' or input_assisted!= 'a'){
		displayText("\nif you want to disable the assisted driving press a or p if you want to exit\n", TEXT_DELAY);
		std::cin >> input_assisted;
	
	if(input_assisted == 'a'){
		manual = true;
		assisted = false;
		manualDriving();
		break;
	}
	else if(input_assisted == 'p'){
	  ros::shutdown();
	}
	else{
	if (minDistances.left <= brakeThreshold) {
    		newVel.linear.x = velFromTeleop.linear.x/2;
    		newVel.angular.z = 1; // Turn the other way
    		
  	} else if (minDistances.right <= brakeThreshold) {
    		newVel.linear.x = velFromTeleop.linear.x/2;
    		newVel.angular.z = -1; // Turn the other way

  	} else {
    		displayText("\nlistening for commands",TEXT_DELAY);
  	}

  	pubCmdVel.publish(newVel);
	}
  	
	
	}
  	
}

/*
* Function for modify the velocity that will be usefull for the collision avoidance
* VelFromTeleop will be modified only if assisted is true
*/
void ControllerClass::UserDriveCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
	
	if (assisted == false){
		
		//std::cout<< "LINEARE %s\n", std::to_string(msg->linear.x);
		//std::cout<< "ANGOLARE %s\n",std::to_string(msg->angular.z);
		pubCmdVel.publish(msg);
	}else{
		
		velFromTeleop.linear.x = msg->linear.x;
  	        velFromTeleop.linear.z = msg-> angular.z; // Save new velocity as class variable
	}
  	
}

/*
* Function for handling the completeness of the action or the timeout of the same
*/

void ControllerClass::sendInfo(bool temp){
	if(temp == true){
		//Action finished before the timeout
		isArrived = true;
	}else{
		//time Expired
		displayText("timeexpired", TEXT_DELAY);
	}
  	std_msgs::Bool msg;
  	msg.data = temp;
  	pubStateInfo.publish(msg);
  }	
	


/*
* Function for canceling the goal by subscribing to a specific topic.
* it depends on the input from the user
*/
void ControllerClass::CancelCallBack(const std_msgs::String &msg){
    
    
    if( msg.data.compare("cancel") == 0){
    	
    	ac.cancelGoal();
    	clearTerminal();
    	terminalColor('4');
    	displayText("The goal has been canceled", TEXT_DELAY);
    	ros::shutdown();
    }
    
}

/*
*Starting Function for handling the program
*/
void ControllerClass::mode_choice(){
	if(!node_handle.hasParam("rt1a3_action_timeout")){
  		node_handle.setParam("/rt1a3_action_timeout", ACTION_TIMEOUT_DEFAULT);
  	}
  	
 	 //Set the brake threashold on the parameter server so it can be tweaked in runtime
 	 if(!node_handle.hasParam("rt1a3_brake_threshold")){
 	 	node_handle.setParam("/rt1a3_brake_threshold", BRAKE_THRESHOLD_DEFAULT);
  	}
  	
  	if(current_mode==1){
  		autonomousDriving();
  	}else if(current_mode==2){
  	
  		manualDriving();
  	} else if(current_mode == 3){
  		
  		collisionAvoidance();
  	} else 
  		displayText("mode not correct, please check your input",TEXT_DELAY);


}
int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "final_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  
  ros::CallbackQueue firstQueue;
  nh2.setCallbackQueue(&firstQueue);
  firstQueue.callAvailable(ros::WallDuration());
  
  ros::AsyncSpinner spinner(0);
  ros::AsyncSpinner spinner2(0,&firstQueue);
  ControllerClass controller_object(&nh, &nh2);
  spinner.start();
  spinner2.start();
  
  controller_object.mode_choice();
  
  ros::waitForShutdown();

  return 0;
}









    
