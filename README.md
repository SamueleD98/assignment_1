# ExpRobLab_FirstAssignment

**A ROS-based exercise for the Experimental Robotics Laboratory course held at the University of Genoa.**  
Author: *Samuele Depalo*

---

## Introduction

This repository contains ROS-based software for controlling a robot. The main component is a state-machine that represent the robot behaviour according to the environment and to some stimulus.

### Scenario

The considered mobile robot retrieves environment related information from an ontology and uses this data for moving across locations. 
Here how the robot should behave:
1. The robot should move between corridors
2. If a room is not visited for a given time, the robot should visit it
3. If the robot's battery is low on energy, the robot should stop the current action and go in a specific room for recharging.  

### Package List

The repository contains the following resources:
- action --> actions' structure
- launcher --> ROS launchers
- msg --> messages' structure 
- ontology --> .owl files
- scripts --> python scripts for the ROS nodes
- srv --> services' structure
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

list of messafes and parameters

## Installing and running
In order to install and run this application, first you should install the *aRMOR* and the *SMACH* package (you can follow the procedure described [here](https://unigeit.sharepoint.com/sites/106723-ExperimentalRoboticsLaboratory/Class%20Materials/Forms/AllItems.aspx?id=%2Fsites%2F106723%2DExperimentalRoboticsLaboratory%2FClass%20Materials%2FROS%2Dinstallation%2Emd&parent=%2Fsites%2F106723%2DExperimentalRoboticsLaboratory%2FClass%20Materials)). Mind that the software also exploits [roslaunch](http://wiki.ros.org/roslaunch), [rospy](http://wiki.ros.org/rospy) and [actionlib](http://wiki.ros.org/actionlib/DetailedDescription).

For running the software call the launcher with `roslaunch assignment_1 system.launch`. This will set the parameters in the server, run the aRMOR server, the state machine and all the other necessary components later described.  
The parameters you can tune for testing the software are later described.

## Code description

### Software components

It follows the details of each software component implemented in this repository, which is available in the scripts/ folder.

*The State Machine node*


*The Ontology Interface node*
*The Robot State node*
*The Scanner node*
*The Planner node*
*The Controller node*



with screenshot/gifs

## Working hypothesis and environment
### System's features
at which rate the system can stand the change in the battery status
modularity
parametrization
even the recharging can be interrupted
### System's limitations
### Possible technical improvements

## Contact me
Samuele Depalo  
Personal mail: [depalo.samuele@gmail.com](depalo.samuele@gmail.com)  
Istitutional mail: [s5153930@studenti.unige.it](s5153930@studenti.unige.it)
