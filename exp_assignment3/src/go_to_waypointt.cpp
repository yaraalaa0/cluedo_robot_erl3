/**
 * \file go_to_waypoint.cpp
 * \brief this file is an implementation of the go to waypoint PDDL action
 * \author Yara Abdelmottaleb
 * \version 0.1
 * \date 30/06/2022
 *
 * \details
 *
 * Clients: <BR>
 *   reaching_goal
 *  
 *   
 *
 * Description :
 *
 * This node implements the interface for the PDDL action go to waypoint
 * It drives the robot to a given waypoint
 * 
 *
 *
*/

#include "exp_assignment3/go_to_waypointt.h"
#include <unistd.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient ac("move_base", true);

namespace KCL_rosplan {
        /**
	 * \brief this is the initialization function of GoToWaypointInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	GoToWaypointtInterface::GoToWaypointtInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	
	/**
	 * \brief this is the callback function for the go to waypoint action interface
	 * 
	 *
	 * \return true when the action is complete
	 *
	 * This function gets the named goal waypoint and calls the action service with the corresponding goal position to drive the robot to this waypoint
	 * After reaching the goal, it returns true
	 * 
	*/
	bool GoToWaypointtInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		move_base_msgs::MoveBaseGoal goal;
		
		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "odom";
		goal.target_pose.header.stamp = ros::Time::now();
		
		
		if(msg->parameters[1].value == "wp1"){
		goal.target_pose.pose.position.x = -4.0;
		goal.target_pose.pose.position.y = -3.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp2"){
		goal.target_pose.pose.position.x = -4.0;
		goal.target_pose.pose.position.y = 7.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp3"){
		goal.target_pose.pose.position.x = 5.0;
		goal.target_pose.pose.position.y = -7.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp4"){
		goal.target_pose.pose.position.x = 5.0;
		goal.target_pose.pose.position.y = -3.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp5"){
		goal.target_pose.pose.position.x = 5.0;
		goal.target_pose.pose.position.y = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp6"){
		goal.target_pose.pose.position.x = -5.0;
		goal.target_pose.pose.position.y = 2.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp0"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -1.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		
		ac.sendGoal(goal);
		ac.waitForResult();
		
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		  ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		  return true;
		  }
		else{
		  ROS_INFO("Action (%s) failed", msg->name.c_str());
		  return false;
		}
		
	}
}


        /**
         * \brief this is main function of the node
         * It initializes the node handle, the action interface, and the service clients 
         *
        */
	int main(int argc, char **argv) {
		ros::init(argc, argv, "GoToWaypointt_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::GoToWaypointtInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
