#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_action_interface/RPActionInterface.h"



namespace KCL_rosplan {

	class GoToWaypointtInterface: public RPActionInterface
	{

	private:

	public:

		/* constructor */
		GoToWaypointtInterface(ros::NodeHandle &nh);
		
                //tell the action client that we want to spin a thread by default
                //MoveBaseClient ac("move_base", true);
                
		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

