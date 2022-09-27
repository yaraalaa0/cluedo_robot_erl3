# cluedo_robot_erl3
Third assignment of Experimental Robotics Laboratory course, M.Sc. in Robotics Engineering, University of Genova, Italy

This is a ROS implementation of a robot agent playing a simplified Cluedo Game collecting hints in the form of Aruco markers and checking hypotheses. The robot's actions is planned using PDDL and executed using ROSPlan interfaces. The robot's knowledge is represented in OWL ontology that is being accessed using ARMOR client. The hints in the environment are represented in the form of Aurco markers

The system was implemented and tested on the [docker image](https://hub.docker.com/repository/docker/carms84/exproblab) provided by Prof. Carmine Recchiuto, University of Genova, Italy

|       Author Name          | Student ID |      Email Address       |
| :------------------------: | :--------: | :----------------------: |
|     Yara Abdelmottaleb     |  5066359   |  [yara.ala96@gmail.com](mailto:yara.ala96@gmail.com)   |


## Introduction:

This is a ROS implementation of a robot agent playing a simplified Cluedo Game collecting hints in the form of Aruco markers on the ground and checking hypotheses. The agent goes randomly to one of six locations in the environment to collect hints in the form of *(who, PERSON)*, *(where, PLACE)* and *(what, WEAPON)*. Collected hints are added to the ontology and after 3 hints or more, are collected the agent goes to the center point to check if a correct hypothesis was found yet or not. The agent continues to explore the environment, collect hints, and check hypotheses until it finds the correct hypothesis.

## Component Diagram:

The software architecture of the system is composed of 13 main components: 

- **The knowledge base (ontology)**: this is the OWL ontology representing the current knowledge of the robot agent. In the beginning it contains the class definitions of `HYPOTHESIS`, `COMPLETE`, `INCONSISTENT`, `PERSON`, `PLACE`, and `WEAPON`, as well as the object properties definitions of *(who, PERSON)*, *(where, PLACE)*, and *(what, WEAPON)*. As the robot explores the environment, new individuals and proberty assertions are added to the ontology.
- **ARMOR**: the armor service responsible for connecting with the knowledge base for querying the ontology or updating it. It is fully implemented by [EmaroLab](https://github.com/EmaroLab/armor). In this project, it is mainly used by the ontology server for adding new hypotheses and hints, and querying the individuals of COMPLETE hypothesis class.
- **Ontology Server**: a ROS server implemented to handle the communication with AROMR. It can receive two types of service requests: adding hints to the ontology through the service `/add_hint` as a `erl2/Hint.h` message, and getting the list of current complete hypotheses through the service `/check_hyp_complete` as a `erl2/HypCompCheck.h` message. 
- **Simulation Oracle**: a ROS node representing the Oracle of the game. It implements a service `/oracle_hint` that accepts as a request the aruco marker ID and replies with the corresponding hint in the form of `erl2/ErlOracle.h` message. The returned hint can contain valid or non-valid values. It also offers a service `/oracle_solution` to send the correct hypothesis ID as a `erl2/Oracle.h` message given an empty request.
- **ROSPlan**: a ROS package responsible for problem generation, planning, and plan execution given a problem and domain PDDL files.
- **Task Manager**: The main node that calls the ROSPlan services: problem generation, planning, parse plan, and plan dispatch. If the plan execution fails, it updates the current knowledge state based on the last dispatched action and re-plan.
- **GoToWaypoint Action Interface**: The ROSPlan action interface for the PDDL action `goto_waypoint` responsible for moving the robot base from one waypoint to another. Waypoints can be one of seven: `wp0`: (0 , -1), `wp1`: (-4 , -3), `wp2`: (-4 , 2), `wp3`: (-4 , 7), `wp4`: (5 , -7), `wp5`: (5 , -3), and `wp6`: (5 , 1). It sends the goal to `Move Base` action server and waits until it is reached.
- **Move Base Action Server**: a ROS action server that plans a path and drives the robot towards a given target pose given the map of the environment without colliding with any of the obstacles. 
- **GetTwoHint Action Interface**: The ROSPlan action interface for the PDDL action `get_two_hint` responsible for getting two aruco marker IDs from the surrounding room and get their corresponding hints through the service `/oracle_hint` and checking the validity of the received hints. If there is a newly received hint and it is valid (the key and value are not empty nor equal -1), it calls the service `/add_hint` to add the received hint to the ontology.
- **Hint Server**: The server responsible for exploring the environment and looking for any black objects (aruco markers) to perform visual servoing. When it detects a black object, it approaches it until the marker is in the center of the image and its size is big enough. Then, it keeps sending images of the marker as a service request to `Marker Publisher` through the service `/get_aruco_code` until it receives a valid marker ID as a response (because sometimes the `Marker Publisher` doesn't recognize the marker ID from the first image). 
- **Marker Publisher**: The server implementing the service `/get_aruco_code` responsible for receiving images of the aruco marker and replying with the corresponding marker ID using `aruco` library and the given 40 aruco markers models. It was modified from the original repository [aruco_ros](https://github.com/CarmineD8/aruco_ros) 
- **CheckHypothesisCorrect Action Interface**: The ROSPlan action interface for the PDDL action `check_hypothesis_correct` responsible for checking if one of the collected hypotheses is the correct one or not. It calls the service `/check_hyp_complete` to get the list of complete hypotheses IDs, and it calls the service `/oracle_solution` to get the correct hypothesis ID. It, then, checks if one of the complete hypotheses is the correct one or not.


![alt text](https://github.com/yaraalaa0/cluedo_robot_erl3/blob/main/cluedo3_comp_diag.jpg?raw=true)


## State Diagram:

The agent has four possible states:
- **GoingToRandomPlace:** the robot is going to a random waypoint for exploration
- **GettingHints:** the robot is checking for hints in the place it is currently in
- **GoingToCenter:** the robot is going to the center waypoint
- **CheckingHypothesis:** the robot is checking whether one of its current collected hypotheses is the correct one

There are, also, four possible events (state transitions):
- **reached:** indicating that the robot reached its target position
- **got a hint:** indicating that the robot received a hint (whether it is valid or not)
- **collected 3 hints:** indicating that the robot collected 3 hints at least
- **hyp_non_correct:** indicating that the robot checked the current hypotheses and none of them was correct.
 

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/cluedo2_state_diag.jpg?raw=true)


## Installation and Running Procedures:

To run the program, you need first to install [ARMOR](https://github.com/EmaroLab/armor) in your ROS workspace.

Then, you need to adapt the code in armor_py_api scripts to be in Python3 instead of Python2:
  - add "from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError" in armor_client.py
  - replace all "except rospy.ServiceException, e" with "except rospy.ServiceException as e"
  - modify line 132 of armor_query_client with: "if res.success and len(res.queried_objects) > 1:"

Add the path of the armor modules to your Python path:
```
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```
Download this repository (contains two packages) to your workspace. Add [erl2](https://github.com/yaraalaa0/cluedo_robot_erl2) package to the same workspace. Then, build everything

```
catkin_make
```


To launch the program, run the following commands in order on five terminal tabs:
- launch ROSplan with the action interfaces: 
```
roslaunch exp_assignment3 rosplan_cluedo.launch
```
- Launch ARMOR:
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
- Launch the simulation:
```
roslaunch exp_assignment3 simulation.launch
```
- Launch the ontology server, hint server, and marker identifier server:
```
roslaunch exp_assignment3 servers.launch
```
- Run the task manager to start the game:
```
rosrun exp_assignment3 task_manager.py
```
The actions and received hints are displayed on the first terminal. The detected markers are displayed on the fourth terminal. The plan success result is displayed on the fifth terminal.

## Result:
**A video that shows the system running can be found [here](https://drive.google.com/file/d/13Kbp7WCyotH63hXQO8XWdrwkm3gZRxAm/view?usp=sharing)**

The video shows the robot first going to wp1. Then, it starts looking for hints. It detects the first marker and then it continues exploration until it gets the second marker. Then, it gets the hints corresponding to these marker IDs. After that, it goes to another waypoint to collect other two hints. Then, it goes to the center point to check if a correct hypothesis was collected or not. If not, it continues exploration until it finds a correct one. 
