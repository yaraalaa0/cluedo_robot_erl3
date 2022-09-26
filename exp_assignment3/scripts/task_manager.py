
#!/usr/bin/env python

"""
.. module:: task_manager.py
   :platform: Unix
   :synopsis: this file is an implementation of the task manager node
   
.. moduleauthor:: Yara Abdelmottaleb
 
This node implements the task manager. 
It calls the problem generator, the planning server, the plan parser, and the plan dispatcher to start the execution of the PDDL task plan.
It checks on the goal success result. If the goal wasn't successful, it updates the current knowledge and calls the problem generator, the planning server, the plan parser, and the plan dispatcher again.
It keeps doing that until the goal is achieved successfully
 
Subscribes to:
   /rosplan_plan_dispatcher/action_dispatch
 
Clients:
   /rosplan_problem_interface/problem_generation_server
   /rosplan_planner_interface/planning_server
   /rosplan_parsing_interface/parse_plan
   /rosplan_plan_dispatcher/dispatch_plan
   /rosplan_knowledge_base/update
   /rosplan_knowledge_base/update_array
  
"""

import roslib
import rospy
from std_srvs.srv import Empty, EmptyRequest
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceRequest, DispatchServiceResponse
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from diagnostic_msgs.msg import KeyValue


last_dispatched_action = "" # a variable to store the last dispatched action name to recognize the action that caused the plan failure

def clbk_ac_dispatch(msg):
    """
    This is the callback function for /rosplan_plan_dispatcher/action_dispatch subscriber
    It receives the last dispatched action from the plan and stores it in the global variable last_dispatched_action to be checked later if the plan execution fails
    If the dispatched action is get_hint, it increases the hints_received global variable
    
    Args:
      msg(ActionDispatch): the dispatched action
    
    """
    global last_dispatched_action
    last_dispatched_action = msg.name
    

def main():
    """
    This is the main function of the node
    It initializes the node handle, the action dispatch subscriber, and the service clients /rosplan_problem_interface/problem_generation_server , /rosplan_planner_interface/planning_server,
    /rosplan_parsing_interface/parse_plan, /rosplan_plan_dispatcher/dispatch_plan , /rosplan_knowledge_base/update , and /rosplan_knowledge_base/update_array
    
    It calls the problem generator, the planning server, the plan parser, and the plan dispatcher to start the execution of the PDDL task plan.
    It checks on the goal success result. If the goal wasn't successful, it updates the current knowledge and calls the problem generator, the planning server, the plan parser, and the plan dispatcher again.
    It keeps doing that until the goal is achieved successfully
    
    """
    global last_dispatched_action
    
    # initialize the ROS node
    rospy.init_node('task_manager')
    
    # Initialize the rosplan services clients
    problem_client = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    planning_server_client = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    parser_client = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    dispatcher_client = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)
    
    update_client = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    arr_update_client = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    action_dispatch_sub = rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',ActionDispatch, clbk_ac_dispatch)
    
    # Call services 
    empty_req = EmptyRequest()
    problem_client(empty_req)
    planning_server_client(empty_req)
    parser_client(empty_req)
    req = DispatchServiceRequest()
    goal_success = dispatcher_client(req)
    print("Goal Success Result:")
    print(goal_success.success)
    print("Goal Achievement Result:")
    print(goal_success.goal_achieved)
    
    
    
    while (goal_success.success == False):
        if last_dispatched_action == "check_hypothesis_correctt":
            print("check_hypothesis_correct action failed!")
            
            # update the current state of the system
            
            upd_req = KnowledgeUpdateServiceRequest()
            upd_req.update_type = 2
            upd_req.knowledge.knowledge_type = 1
            upd_req.knowledge.attribute_name = "correct_hyp"
            update_client(upd_req)
            
            
            # re-plan
            problem_client(empty_req)
            planning_server_client(empty_req)
            parser_client(empty_req)
            
            req = DispatchServiceRequest()
            goal_success = dispatcher_client(req)
            print("Goal Success Result:")
            print(goal_success.success)
            print("Goal Achievement Result:")
            print(goal_success.goal_achieved)
            
    
if __name__ == '__main__':
    main()
