/**
* \file state_machine.cpp
* \brief This file contains a state machine to start or stop the random beahvior of the robot
* \author Maria Luisa Aiachini
*
* \details
*
* Services: <BR>
* 	°/user_interface
*
* Clients: <BR>
*	°/position_server
*
* Action Client: <BR>
*	°/go_to_point
*
* Publishes to: <BR>
*	°/reached
*	°/duration
*
* Description:
*
* This node is used for starting or stopping the robot random behavior.
* It receives a request from the /user_interface with a command: 'start' or 'stop'.
* If the robot needs to start, the node sends a request to the /position_server with
* the min and the max values to obtain a position that will be reached.
* When the node receives a response it sends the target position to the /go_to_point server
* that will make the robot move; at the same time a timer is started to compute the total
* time needed to reach the goal.
* Once the robot reaches the goal, this information is published on the topic /reached 
* and the time needed is computed and published on the topic /duration.
* Finally, if the node has not received a 'stop' command from the /user_interface, another
* target is computed and the behavior will start again.
* If the node receives a 'stop' command at any time, the goal is cancelled and the robot
* is stopped immediately.
*
*/


#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/PlanningAction.h"
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"


bool start = false;
bool stopped = true;
float total_seconds = 0;
std_msgs::Bool reached;
ros::Time start_timer;
ros::Time stop_timer;
std_msgs::Duration tempo;

/**
* \brief
*
* \param
* \param
*
* \return
*
*
*/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
* \brief
*
* \param
* \param
*
* \return
*
*
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::NodeHandle n1;
   ros::NodeHandle n2;
   ros::NodeHandle n3;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   // Defining the action client
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/go_to_point");
   
   ros::Publisher pub = n2.advertise<std_msgs::Bool>("/reached",1000);
   ros::Publisher time_pub = n3.advertise<std_msgs::Duration>("/duration",100);
   // Defining the max and min values for x and y to compute the random in between
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   while(ros::ok())
   {
   	ros::spinOnce();
   	if (start)
   	{
   		if (stopped == true) 
   		{
		// Calling the random position service to obtain the target position and orientation
   		client_rp.call(rp);
   		rt2_assignment1::PlanningGoal goal;
   		goal.target_pose.pose.position.x = rp.response.x;
   		goal.target_pose.pose.position.y = rp.response.y;
   		goal.target_pose.pose.orientation.z = rp.response.theta;
   		
   		std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
   		start_timer = ros::Time::now();
   		ac.sendGoal(goal);
   		
   		stopped = false;
		}   
		else 
		{	// Wnhen the targer is reached it is printed to the terminal
			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				stopped = true;
				std::cout <<"Target reached" << std::endl;
				reached.data = true;
			ros::Duration total_time = ros::Time::now() - start_timer;
				tempo.data = total_time;
				time_pub.publish(tempo);
				pub.publish(reached);
			}
		}
   	}
   	else
   	{	// When the user wants to stop the robot the goal is cancelled
   		if (stopped == false)
   		{
   			ac.cancelAllGoals();
   			std::cout <<"Robot stopped"<< std::endl;
   			stopped = true;
   			reached.data = false;
   			pub.publish(reached);
   		}
   	}
   }
}
