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



## Software architecture 

The repository structure is:
- action --> actions' structure
- launcher --> ROS launchers
- msg --> messages' structure 
- ontology --> .owl files
- scripts --> python scripts for the ROS nodes
- srv --> services' structure

There are also files related to the ROS architecture (*CMakeLists.txt* and *package.xml*) and to the code documentation (*Makefile*, *conf.py* and *index.rst*).

### The finite state machine
Here a representation of the implemented state machine:
![state_machine](images/state_diagram.png)  
For semplicity the connection among states don't show the stimulus name but, instead, it's possible to distinguish by the color the normal flow of the machine (black arrows) from the stimulus due to a change in the battery status (red arrows). 

Follows a comment on the machine.
For best practice the machine has the least number of (main) states for the desired behavior, 3:
1. A **Mapping** state: here the information about the environment (.owl file) is loaded to be avaiable at need.
2. A **Monitoring** state: the robot is expected to move across rooms and observe, *monitor*, the environment (although in this work the monitoring task consist of busy waiting).
3. A **Recharge** state: once the system is notified the battery is low, the robot should move to a specific location for recharging  

Of course, the motion between locations is not instantaneous and so it should be considered a state. This state, though, should have a different behaviour depending on the situation (monitoring/recharging) and in order to reduce the complexity of the execution



[Original](https://github.com/CarmineD8/python_simulator) one developed by [Student Robotics](https://studentrobotics.org/) 
temporal diagram
states diagram
each diagram, comment and list of messafes and parameters

## Installing and running

## Code description
with screenshot/gifs

## Working hypothesis and environment
### System's features
### System's limitations
### Possible technical improvements

## Contact me
