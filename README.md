# ExpRobLab_FirstAssignment

**A ROS-based exercise for the Experimental Robotics Laboratory course held at the University of Genoa.**  
Author: *Samuele Depalo*

---

## Introduction

This repository contains ROS-based software for controlling a robot. The main component is a state-machine that represent the robot behaviour according to the environment and to some stimulus.

### Scenario

The considered mobile robot retrieves environment related information from an ontology and uses this data for moving across locations. 
Here how the robot should behave:
1. The robot should load a map before any other action
2. The robot should move between corridors
3. If a room is not visited for a given time, the robot should visit it
4. If the robot's battery is low on energy, the robot should stop the current action and go in a specific room for recharging.  

### Assumptions

### Package List

The repository contains the following resources:
- action --> actions' structure
  - OICommand.action: It defines the goal and results concerning the ontology interface
  - Scanner.action: It defines the goal and result concerning the map scanning/loading
  - Plan.action: It defines the goal and results concerning motion planning
  - Control.action: It defines the goal and results concerning motion controlling
- launcher --> ROS launchers
- msg --> messages' structure 
  - Point.msg: It is the message representing a 2D point
- ontology --> .owl files
  - map1.owl: ontology as described in the assignment
- scripts --> python scripts for the ROS nodes
- srv --> services' structure
  - GetPose.srv: It defines the request and response to get the current robot position
  - SetPose.srv: It defines the request and response to set the current robot position
- images --> images shown in this ReadMe

There are also files related to the ROS architecture (*CMakeLists.txt* and *package.xml*) and to the code documentation (*Makefile*, *conf.py* and *index.rst*).



## Software architecture 



### The finite state machine
Here a representation of the implemented state machine:
![state_machine](images/state_diagram.png)  
In order to simplify the diagram, the connection among states don't show the stimulus name but, instead, it's possible to distinguish by the color the normal flow of the machine (black arrows) from the stimulus due to a change in the battery status (red arrows).  

The desired behaviour can be easily implemented using three (main) states:
1. A **Mapping** state: here the information about the environment (.owl file) is loaded to be avaiable at need.
2. A **Monitoring** state: the robot is expected to move across rooms and observe, *monitor*, the environment (although in this work the monitoring task consist of busy waiting).
3. A **Recharging** state: once the system is notified the battery is low, the robot should move to a specific location for recharging  



The particularity of this state machine is its concurrent, hierarchical structure. In fact, both the Monitoring and the Recharging state consist of three inner states:
1. **Move** state: it retrieves the next target location for the robot and interacts with the planner, and then the controller, in order to reach that location.
2. **Monitor**/**Recharge** state: in this implementation these states actually consist in busy waiting for a given time, simulating the actual task the robot should carry in that time (either exploring the environment or recharging itself).
3. **Check Battery status** state: this is a *Monitor* state, a particular kind of states that *Smach* allows to use. It works in *concurrence* with the other state machine (consisting of the two previosuly described states) waiting for a message to be published in a specific topic: '/battery_status'. Once something is published a callback is called to decide if due to this change in the battery level, the current state machine should be preempted for the other main state.  

**Why not having a main MOVE state?**  
The motion between locations could be considered a single main state. This state, though, should have a different behaviour depending on the situation (monitoring/recharging) in both choosing the next location and in terms of being preempted when a stimulus arrives (if the robot is already going to the recharging room and a *battery low* signal arrives, the robot shouldn't stop the motion for that room a start it from the beginning). So, implementing two different **move** states reduce consistently the complexity of the execution and, more, increase the modularity (what if I want one of the monitor states to monitor a topic and the other state a different one?). The code is however simple: they are created from the same class ( *Move* ) but with a slightly different configuration (more of this later).  

**Why concurrent?**  
With this concurrent structure, the transitions among states are guaranteed to be executed as soon as a stimulus arrives, and this without using any mutex of the sort. 

**Why hierarchical?**  
Choosing of separating the motion of the robot from the main task of the two states (Monitoring and Recharge) increases the modularity: in fact, doing so, allow to change one of the two tasks by just modyfing the inner state and leaving as it is the other. Even more, by adding another inner state you can easily increase the tasks of the robot.

**Temporal diagram**  
Here how the state machine evolves in time:  
![temporal_diagram](images/temporal_diagram.png)  
Normally the robot should keep moving from location to location.  
When a *battery low* signal is received, the **Monitoring** state is preempted and the execution goes to the **Recharging** state. From there, either the robot goes in the recharging room a wait for itself to be fully recharged or, could happen, a new signal comes (*battery high*) before it could even reach the room. In that case it's the **Rechargin** state to be preempted for the Monitoring one: there's no point in going to recharge it the robot has still power. Why the battery should result full after a *battery low* signal comes is not part of the discusion (could be a battery level misreading,  poor/defective hardware,.. ).

### Software components

It follows the details of each software component implemented in this repository, which is available in the scripts/ folder.

#### The State Machine node  
It implements the behaviour the robot follows.  
Four different kind of state's implementation are described in as many classes: *Mapping()*, *Move()*, *Monitor()* and *Recharge()*.    
The execute of a Mapping() state simply send a goal to the Scanner node for loading the map and waits for it to end.  
Move(), depending on the "type" argument, either asks to the Ontology Interface node for the next room to visit or it asks for the recharging room. After, it send a goal first to the planner and then to the controller for planning and control the motion to the target. Finally, it asks the Ontology Interface to update the robot position in the ontology.  
In Monitor() and Recharge() there's only a busy waiting, to simulate the time the robot should spend performing those actions.  
The main code consist in setting the node, configuring the state machine as already described, starting the server for visualization, initializing the action clients, waiting for the actions servers and finally executing the state machine.  
Important to mention three callbacks:
- monitor_cb() is called when a new message is published in /battery_status and terminates its execution only if the new battery status is different from the old one. This in order to ignore consecutive messages with the same information (if the robot already knows the battery is low and it's going to recharge, it should not interrupt this action if a new *battery low* message arrives)
- child_term_cb() is called when any of the concurrence states terminates and simply terminates all the other states. This because, having a monitor state (which terminate its execution when the battery status changes) in concurrence with the monitoring/recharging states, the desired behavior consist in preempting one of these states when the other terminates (i.e. the battery status goes to low --> the check_battery_status state terminates its execution --> preempt the monitoring states for recharging, or, viceversa, the battery status goes to high --> the check_battery_status state terminates its execution --> preempt the recharging states for monitoring).
- out_cb_monitoring() is called when all the concurrent states are terminated and decides which outcome the concurrence state machine, in the monitoring state, should return. If the inner *EXECUTE* state machine has been preempted from the *Check Battery status* then the outcome is *battery_low*, else the outcome is *monitoring_done*.
- out_cb_recharge() is similar to the precedent. In this case, though, it always returns *recharge_done* because either the robot terminate the recharging execution or the state is preempted because a *battery high* update is received.

Actions:  
- *OntologyInterface*, client
- *action_scanner*, client
- *action_planner*, client
- *action_controller*, client


#### The Ontology Interface node  

This node provides an interface for all the other components to the armor server, allowing them to query and manipulate an ontology in a easier and modular way. By doing so, the other components (e.g. planner, controller) have no commands related to the armor server connection.  
The node presents a *SimpleActionServer* which possible goals are:
- *load_map*: load the ontology specified in the rosparams server, save lists of names of the main locations and also the name of the robot, update the urgency threshold with the one given as parameter, call the reasoner and update the robot's *now* timestamp.
- *next_room*: find the next location the robot should visit following a predetermined algorithm. It retrieves the urgent rooms as the elements that are both in the urgent locations list and in the rooms list, then takes one of those which are also reachables. If there are none, it choose a reachable corridor. If no corridors nor urgent rooms are avaiable, it takes randomly a reachable location.
- *move_to*: once the robot reaches a new location, the node update both its position in the ontology and the *visitedAt* value for the new location.
- *recharge_room*: return the location for the recharging of the robot.  

The functions pretty much corresponds to the actions the server is able to carry out. There are two more functions which are often called during the computation:
- *update_timestamp*: like already mentioned it updates the *now* timestamp of the robot and calls the reasoner.
- *clean_response_list*: the responses returned by the armor queries have to be processed before using them. Firstly the function retrieves a list from the response and then removes the *IRI* plus some special character from each element. When it's dealing with a timestamp, it removes also the string '^^xsd:long'.  

Services:
- /armor_interface_srv (waits for it to be ready)

Actions:
- OICommandAction, server



#### The Robot State node  
Similar to the one implemented in [arch_skeleton](https://github.com/buoncubi/arch_skeleton), this node implements two services (set_pose and get_pose) and a publisher (battery_status).

The services allow setting and getting the current robot position, which is shared between the planner and the controller. 

The battery_status message is published when the batter changes state. We consider two possible states: low battery (True is published) and recharged (False is published).
The battery_time parameter is used to delay the published messages.

Messages:
- battery_status, publisher

Services:
- GetPose, server
- SetPose, server

#### The Scanner node
This node simulates a scanner which should retrieve information about the environment (e.g. from a QR code). 
For now it just send a goal to the Ontology Interface node for loading the ontology passed as parameter.  

Actions:
- ScannerAction, server
- OICommandAction, client

#### The Planner node  
A simplification of the one implemented in [arch_skeleton](https://github.com/buoncubi/arch_skeleton), the behaviour remains pretty much the same, for semplicity the feedbacks were removed.

Actions:
- PlanAction, server

Services:
- GetPose, client


#### The Controller node  
A simplification of the one implemented in [arch_skeleton](https://github.com/buoncubi/arch_skeleton), the behaviour remains pretty much the same, for semplicity the feedbacks were removed.

Actions:
- ControlAction, server

Services:
- GetPose, client
- SetPose, client

## Launching the Software

### Dependencies

In order to install and run this application, first you should install the *aRMOR* and the *SMACH* package (you can follow the procedure described [here](https://unigeit.sharepoint.com/sites/106723-ExperimentalRoboticsLaboratory/Class%20Materials/Forms/AllItems.aspx?id=%2Fsites%2F106723%2DExperimentalRoboticsLaboratory%2FClass%20Materials%2FROS%2Dinstallation%2Emd&parent=%2Fsites%2F106723%2DExperimentalRoboticsLaboratory%2FClass%20Materials)). Mind that the software also exploits [roslaunch](http://wiki.ros.org/roslaunch), [rospy](http://wiki.ros.org/rospy) and [actionlib](http://wiki.ros.org/actionlib/DetailedDescription).

### Installation

Follow these steps to install:
- Clone this repository inside your ROS workspace (which should be sourced in your .bashrc)
- Run chmod +x <file_name> for each file inside the scripts folder.
- Run catkin_make from the root of your ROS workspace.

### Launchers

For running the software call the launcher with `roslaunch assignment_1 system.launch`.  
This will set the parameters in the server, run the aRMOR server, the state machine and all the other necessary components later described.  

### ROS Parameters  
This software requires the following ROS parameters:
- `O_path`: Path for the desired environment ontology
- `O_IRI`: IRI for the desired environment ontology
- `armor_client_id`, default "client"
- `armor_reference_name`, default "ref"
- `ontology_reasoner`, default "PELLET"
- `urgency_threshold`, time after last visit which makes the location urgent, default 7
- `charging_station_in`, recharging room, default "E"
- `scanning_time`, time before loading the map, default 5
- `planning_time`, time for planning a via point, default between 0.1 and 0.2
- `motion_time`, time for reaching a via point, default between 0.1 and 0.2
- `monitoring_time`, busy waiting duration, default 10
- `recharging_time`, busy waiting duration, default 10
- `battery_time`, battery status toggle time, default between 15 and 40


## RUNNING CODE GIFS



with screenshot/gifs

## Working hypothesis and environment
### System's features
at which rate the system can stand the change in the battery status
modularity
parametrization
even the recharging can be interrupted
### System's limitations
### Possible technical improvements
actions feedback

## Contact me
Samuele Depalo  
Personal mail: [depalo.samuele@gmail.com](depalo.samuele@gmail.com)  
Istitutional mail: [s5153930@studenti.unige.it](s5153930@studenti.unige.it)
