Author: Mattia Piras

#third assignment for Research Track 1: Course of Robotics Engineering Unige  
---------------------------------------------------------------------------------------------------------

In this project, a robot moves in an environment initially unknown to it. Thanks to laser scanners mounted on board the robot is able to build a map of the environment and move without encountering obstacles. It is required that the robot can move in 3 different modes:

    - setting the coordinates of a goal to be achieved and the robot would have to dynamically calculate a path to reach it
    - controlling it via keyboard
    - controlling it via keyboard with assistance to avoid obstacles
    
##INSTALLING AND RUNNING
---------------------------------------------------------------------------------------------------------
Some packages are required; Run the following commands to install them.

- Install x-term:

	$ sudo apt install xterm
	
- Download package:

	$ git clone https://github.com/CarmineD8/slam_gmapping.git
	
- Install ros navigation stack:

	$ sudo apt-get install ros-<your_ros_distro>-navigation

In order to run multiple nodes at the same time, a launch script is provided: rt1a3.launch:

<launch>
  
  <!-- For terminal (human) reading -->
  <env name="ROSCONSOLE_FORMAT" value="${message}"/>
  
  <!-- Essential parameters -->
  <param name="rt1a3_action_timeout" type="double" value="60.0"/>
  <param name="rt1a3_brake_threshold" type="double" value="1.0"/>

  <!-- Essential Nodes -->
  <node name="final_ui" pkg="final_assignment"
    type="final_ui" required="false" output="screen" launch-prefix="xterm -e"/>
  

  <node name="final_controller" pkg="final_assignment" output="screen" launch-prefix="xterm -e"
    type="final_controller"  />
</launch>

The User can launch the simulation_sim.launch, mvoe_base.launch, rt1a3_teleop.launch and rt1a3.launch

##Environment and Mapping
-----------------------------------------------------------------------------------------------------

The robot moves in the environment in the figure (Gazebo view):

<img width="782" alt="gazebo" src="https://user-images.githubusercontent.com/92534255/155940743-e88f5ab1-e097-4025-a3a5-460ad08b761e.png">


Initially the robot does not have a map of the surrounding environment, but can build it thanks to the laser scanner it is equipped with and thanks to the gmapping package. 

The final map, visible on Rviz is as follows:

<img width="487" alt="rviz" src="https://user-images.githubusercontent.com/92534255/155940790-33f4e580-fc23-494b-9d37-f9bdb1d97679.png">



##Project structure
------------------------------------------------------------------------------------------------------

After running rt1a3.launch, several nodes are active:

<img width="698" alt="rqt2" src="https://user-images.githubusercontent.com/92534255/155940813-29d13949-6ece-4888-adce-29b4a70f7460.png">



The \cmd_vel topic of \teleop node is remapped on \controller_cmd_vel. In this way the velocity imposed by the user via keyboard isn't immediately imposed to the robot, but it's controlled by \controller node.

\controller node is also connected to \gazebo and \move_base nodes. It receives the robot's status by \move_base\feedback and publish the goal to reach on \move_base\goal. This node also subscribe on \move_base\goal topic to have a goal feedback. It can cancel current goal using \move_base\cancel topic.

\controller node also receives the laser scanner output on \scan topic by \gazebo node and sends to this node the robot velocity on \cmd_vel topic.


#PSEUSOCODE
----------------------------------------------------------------------------------------------------------

##Controller_node behavior can be explained by this pseudi code:

	if(current_mode==1){
  		autonomousDriving();
  	}else if(current_mode==2){
  		manualDriving();
  	} else if(current_mode == 3){
  		collisionAvoidance();
  	} else 
  		displayText("mode not correct, please check your input",TEXT_DELAY);

	autonomousDriving():
		- wait for the actionServer 
		- setGoal: from the user input
		- sendGoal with the actionClient
		- set timeout
		- wait for results from the action server
			- if in time: publish complete TRUE
			- if not in time : publish timeout FALSE
		
	manualDriving():
		- wait for input
			if 'a': ENABLE ASSISTED DRIVING
			if 'p': EXIT THE PROGRAM
			
	assistedDriving(): aka(collisionAvoidance)
		- check input from the user
		- if input different from 'a' (disable assisted) or 'p' (exit)
			- check distances from laser scann
			- modify vel consequently
			- publishes vel 
			
		-
		
##user_node behavior can be explained by this pseudi code:

	if input_user==1: // Autonomous Driving
		set mode_srv.req = 1;
		ask for x and y for the goal
		set goal_srv.req.x and goal_srv.req.y 
		
	else if input_user==2
		set mode_srv.req = 2;
	
	else if input_user==3
		set mode_srv.req = 3;
		
	client_mode ...
	client_goal ...
	cancel_function(): 
		for cancel the goal:
			check if the time is expired
			    -if so:
			    	ros::shutdown()
			    -otherwise:
			    	if pressed 'p':
			    	publish("cancel");

# What I used:
-----------------------------------------------------------------------------------------------------
 - two classes
 - utils.h for addictional functions
 - Asyncronous spinner for decciding the system how many threads utilizing
 - One waiting queue for increasing the speed of the response for canceling the goal
 - two personal services:
  	- Goal_service to send the desired goal once the coordinates are inserted
  	- Behavior_mode_service to send the proper case of utilization of the program
  
 - parameter server
   
   	  - for the timeout
   	  - for treshold from the obstancles detected
   
 - Pub e Sub:
 
 	  - to send the cancel message from the user
	  - scan laser per avoidance obstacle
	  - controller_cmd_vel which is cmd_vel remapped
	  - for the timeout
	  - for cancel the goal
	  - state info (for lleting the user if the action is complete or timeout)
 
 - Action Client:
 
 	- To handle the autonomous driving, for sending the goal, receiving feedback fromth erobot while reaching the goal, for checking the timeout from the server

# Possible Improvements:
 - Implementation of a queue for letting the user to decide in advance a series of goal for the robot to reach
 - Better use of asyncrhonous spinner
 - Integration of actionserver function for a better response from this one
