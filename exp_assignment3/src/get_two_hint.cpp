/**
 * \file get_hint.cpp
 * \brief this file is an implementation of the get hint PDDL action
 * \author Yara Abdelmottaleb
 * \version 0.1
 * \date 30/06/2022
 *
 * \details
 *
 * Subscribers: <BR>
 *
 *   /oracle_hint
 *
 * Clients: <BR>
 *   /add_hint
 *  
 *
 * Description :
 *
 * This node implements the interface for the PDDL action get hint
 * It gets the hint from oracle and updates the ontology with the received hint if it is valid
 * 
 *
 *
*/

#include "exp_assignment3/get_two_hint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "exp_assignment3/Hint.h"
#include "exp_assignment3/Marker.h"
#include "erl2/Hint.h"
#include <string.h>


ros::ServiceClient client_ont;
ros::ServiceClient client_hint;
ros::ServiceClient client_oracle_hint;


namespace KCL_rosplan {
        /**
	 * \brief this is the initialization function of GetHintInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	GetTwoHintInterface::GetTwoHintInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	
	/**
	 * \brief this is the callback function for the get hint action interface
	 * 
	 *
	 * \return true if there is a received hint, false otherwise
	 *
	 * This function checks if there is a received hint. If yes, it checks if its values are valid. If yes, it updates the ontology with the new hint by calling the service /add_hint
	 * 
	*/
	bool GetTwoHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		exp_assignment3::Hint r;
		std::cout<<"Sent request Hints to the server"<<std::endl;
		client_hint.call(r);
		int id1 = r.response.hintID1;
		int id2 = r.response.hintID2;
		std::cout<<"Received marker IDs"<<std::endl;
		
		exp_assignment3::Marker m;
		m.request.markerId = id1;
		client_oracle_hint.call(m);
		int id = m.response.oracle_hint.ID;
		std::string key = m.response.oracle_hint.key;
		std::string value = m.response.oracle_hint.value;
		std::cout<<"hint1 key: "<<key<<" , value: "<<value<<std::endl;
		if(key != "" && value != "" && key != "-1" && value != "-1"){
		    // update the ontology with the new received hint
		    erl2::Hint h;
		    h.request.ID = std::to_string(id);
		    h.request.key = key;
		    h.request.value = value;
		    std::cout<<"Sent Hint to Ontology"<<std::endl;
		    std::cout<<"ID: "<<id<<std::endl;
		    std::cout<<"Key: "<<key<<std::endl;
		    std::cout<<"Value: "<<value<<std::endl;
		    client_ont.call(h);
		        
		}
		
		// get hint of the second marker ID 
		m.request.markerId = id2;
		client_oracle_hint.call(m);
		id = m.response.oracle_hint.ID;
		key = m.response.oracle_hint.key;
		value = m.response.oracle_hint.value;
		std::cout<<"hint2 key: "<<key<<" , value: "<<value<<std::endl;
		if(key != "" && value != "" && key != "-1" && value != "-1"){
		    // update the ontology with the new received hint
		    erl2::Hint h2;
		    h2.request.ID = std::to_string(id);
		    h2.request.key = key;
		    h2.request.value = value;
		    std::cout<<"Sent Hint to Ontology"<<std::endl;
		    std::cout<<"ID: "<<id<<std::endl;
		    std::cout<<"Key: "<<key<<std::endl;
		    std::cout<<"Value: "<<value<<std::endl;
		    client_ont.call(h2);
		        
		}
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
		
		
		
	}
}


        /**
         * \brief this is main function of the node
         * It initializes the node handle, the action interface, the subscriber, and the service client 
         *
        */
	int main(int argc, char **argv) {
		ros::init(argc, argv, "GetTwoHint_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		client_hint = nh.serviceClient<exp_assignment3::Hint>("/get_hint");
		client_oracle_hint = nh.serviceClient<exp_assignment3::Marker>("/oracle_hint");
		client_ont = nh.serviceClient<erl2::Hint>("/add_hint");
		KCL_rosplan::GetTwoHintInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
