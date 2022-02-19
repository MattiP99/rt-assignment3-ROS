#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
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

#define GOAL_TH 0.5
const int TEXT_DELAY = 25000; // microseconds

static struct minDistances minDistances;

//These values set the ros parameters but actually the ros param server values are used
const double ACTION_TIMEOUT_DEFAULT = 30.0; //seconds
const double BRAKE_THRESHOLD_DEFAULT = 1.0;
move_base_msgs::MoveBaseGoal goal;
// Represents MINIMUM distances of obstacles around the robot
struct minDistances {
  float left;
  float right;
};

//CONSTRUCTOR
ControllerClass::ControllerClass() : node_handle(""),node_handle2(""),node_handle3(""), spinner(0) {
  
  x_goal = 0;
  y_goal = 0;
  ROS_INFO("Init Started");

  spinner.start();
  
  //ACTIONCLIENT
  MoveBaseClient ac("move_base", true);
  
  service_mode = node_handle.advertiseService("/switch_mode", &ControllerClass::switch_mode, this);
  service_goal = node_handle.advertiseService("/set_goal", &ControllerClass::set_goal, this);
  
  ros::SubscribeOptions ops;
  ops.template init<std_msgs::String>("/cancel",1, &ControllerClass::CancelCallBack, this);
  ops.allow_concurrent_callbacks = true;
  subMode = node_handle.subscribe(ops);
   //= node_handle.subscribe("/current_mode",1, &ControllerClass::ModeCallBack, this);
  
  // Create a second NodeHandle
  ros::CallbackQueue secondQueue;
  node_handle2.setCallbackQueue(&secondQueue);
  node_handle2.subscribe("controller_cmd_vel", 10, &ControllerClass::UserDriveCallback,this);

  // Spawn a new thread for high-priority callbacks.
  std::thread prioritySpinThread([&secondQueue]() {
  ros::SingleThreadedSpinner spinner;
  spinner.spin(&secondQueue);
  });
  prioritySpinThread.join();
  
  // Create a third NodeHandle
  ros::CallbackQueue thirdQueue;
  node_handle3.setCallbackQueue(&thirdQueue);
  node_handle3.subscribe("scan", 1, &ControllerClass::LaserScanParserCallBack, this);

  // Spawn a new thread for high-priority callbacks.
  std::thread prioritySpinThread2([&thirdQueue]() {
  ros::SingleThreadedSpinner spinner;
  spinner.spin(&thirdQueue);
  });
  prioritySpinThread.join();
	
	//subCmdVelRemapped = node_handle.subscribe("controller_cmd_vel", 10, &ControllerClass::UserDriveCallback,this);
  //subScanner = node_handle.subscribe("scan", 1, &ControllerClass::LaserScanParserCallBack, this);
  ROS_INFO("Init Finished");
}

//DISTRUCTOR
ControllerClass::~ControllerClass() { ros::shutdown(); }




//service to change mode
bool ControllerClass::switch_mode(final_assignment::Behavior_mode_service::Request& req, final_assignment::Behavior_mode_service::Response& res){
   
  
    //The request is of type int32 (by definition of the service), there is no need to check the type
    if (req.mode >= 1 and req.mode <=3){
        current_mode = req.mode;
        res.success = true;
        ROS_INFO("The current mode is now %d ", current_mode);

        //Different message according to the mode
        if (current_mode ==2 or current_mode==3){
            clearTerminal();
            
            displayText("Mode: ", TEXT_DELAY);
            terminalColor(36, true);
            displayText("Manual Drive\n", TEXT_DELAY);
            terminalColor(37,false);
            displayText("\nKeys to control the robot:\n"
                    "---------------------------\n"
                    "Moving around:\n"
                    "u    i    o\n"
                    "j    k    l\n"
                    "m    ,    .\n"

                    "q/z : increase/decrease max speeds by 10 percent\n"
                    "w/x : increase/decrease only linear speed by 10 percent\n"
                    "e/c : increase/decrease only angular speed by 10 percent\n"
                    "anything else : stop\n", TEXT_DELAY);
        }else{
            printf("\nTo set a goal, call the service /set_goal\n");
        }
    }
    //An invalid interger will no affect the system
    else{
        res.success = false;
        displayText("Please enter an existing mode, i.e. 1, 2 or 3", TEXT_DELAY);
    }
    
    return true;        
}


//service to set goal
bool ControllerClass::set_goal(final_assignment::Goal_service::Request  &req, final_assignment::Goal_service::Response &res){
     goal_is_defined = false;
    
    //The goal is only to be considered if the mode 1 is active
    if (current_mode ==1){

        //update the new goal, regarless where this goal is in the map
        ROS_INFO("request is x=%f and y=%f", req.x, req.y);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.orientation.w = 1.;
        goal.target_pose.pose.position.x = req.x;
        goal.target_pose.pose.position.y = req.y;
        
        x_goal = goal.target_pose.pose.position.x;
        y_goal = goal.target_pose.pose.position.y;
        
        res.success = true;
        ROS_INFO("The goal is now set to (%f, %f): ", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.waitForServer()
        ac.sendGoal(goal,
        	    boost::bind(&ControllerClass::doneCb, this, _1, _2),
                    boost::bind(&ControllerClass::activeCb, this),
                    boost::bind(&ControllerClass::feedbackCb, this, _1));)
        
        goal_is_defined = true;
        
        // If action does not finish before timeout, then cancel it and notify user
  	if (moveBaseAC.waitForResult(ros::Duration(actionTimeout))) {
    		actionlib::SimpleClientGoalState state = ac.getState();
   		 ROS_INFO("Action finished: %s", state.toString().c_str());
    		sendInfo("Goal has been successfully reached.");
 	 } else {
    		ROS_INFO("Action timed out.");
    		sendInfo("Timeout: goal has been automatically cancelled. (Are you sure the requested coordinates were reachable?)");
    		ac.cancelGoal();
    }
    else{
        res.success = false;
        ROS_INFO("Goal definition is only for mode 1");
    }

    return true;
    
}

void Controller_class::currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {

    // Update the goal ID if there is a new goal
    if (GoalID != msg->status.goal_id.id) {
        GoalID = msg->status.goal_id.id;
    }
    
    // If there is a goal check if the robot has reached the goal or is trying to reach the goal for a too long time
    if(goal_is_defined)
    {
    	// Take the current robot position
        float dist_x;
	float dist_y;
	float robot_x = msg->feedback.base_position.pose.position.x;
	float robot_y = msg->feedback.base_position.pose.position.y;
    
	// Compute the error from the actual position and the goal position
	dist_x = robot_x - x_goal;
	dist_y = robot_y - y_goal;

	// The robot is on the goal position
	if (abs(dist_x) <= GOAL_TH && abs(dist_y) <= GOAL_TH)
	{
	     printf("Goal reached\n");
	     ac.cancelGoal();       //NON SO SE è GIUSTO COSI RiGUARDA
	}

    	time_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
        if(time > TIMEOUT)
        {
        	printf("The actual goal can't be reached!\n");
        	ac.cancelGoal(); // Cancel the goal if it can't be reached
        }
    }
}

void ControllerClass::CancelCallBack(const std_msgs::String &msg){
    if( msg.data == "cancel"){
    	ac.cancelGoal();
    	clearTerminal();
    	terminalColor(37,false);
    	displayText("The goal has been canceled", TEXT_DELAY);
    }
    
}



void ControllerClass::init_param(){
	// Set the timeout threshold on the parameter server so it can be tweaked in runtime
  	if(!node_handle.hasParam("rt1a3_action_timeout")){
  		nh.setParam("/rt1a3_action_timeout", ACTION_TIMEOUT_DEFAULT);
  	}
  	
  	//Set the brake threashold on the parameter server so it can be tweaked in runtime
  	if(!node_handle.hasParam("rt1a3_brake_threshold")){
  		nh.setParam("/rt1a3_brake_threshold", BRAKE_THRESHOLD_DEFAULT);
  	}
}



void ControllerClass::LaserScanParserCallBack(const sensor_msgs::LaserScan::ConstPtr& scaninfo) {
 	const int NUM_SECTORS = 2;
 	int numElements;
 	int numElementsSector;
 	float leftDistMin;
 	float rightDistMin;
	
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

void ControllerClass::collisionAvoidance() {
	
  	geometry_msgs::Twist newVel;
  	double brakeThreshold;

  	newVel = velFromTeleop;

  	if (nh.getParam("/rt1a3_brake_threshold", brakeThreshold)) {
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


void ControllerClass::UserDriveCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  	this.velFromTeleop = *msg; // Save new velocity as class variable
}


int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "controller_node");
  	
  ControllerClass controller_object;

  ros::waitForShutdown();

  return 0;
}









