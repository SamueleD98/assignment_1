# ExpRobLab_FirstAssignment

**A ROS-based exercise for the Experimental Robotics Laboratory course held at the University of Genoa.**  
Author: *Samuele Depalo*

---

## Introduction

This repository contains ROS-based software for controlling a robot. The main component is a state-machine that represent the robot behaviour according to the environment and to some stimulus.


### Scenario

The considered mobile robot retrieves envionment related information from an ontology and uses this data for moving across locations. 
Here how the robot should behave:
1. The robot should move between corridors
2. If a room is not visited for a given time, the robot should visit it
3. If the robot's battery is low on energy, the robot should stop the current action and go in a specific room for recharging


## Software architecture 

The repository structure is:
- action --> actions' structure
- launcher --> ROS launchers
- msg --> messages' structure 
- ontology --> .owl files
- scripts --> python scripts for the ROS nodes
- srv --> services' structure

There are also files related to the ROS architecture (*CMakeLists.txt* and *package.xml*) and to the code documentation (*Makefile*, *conf.py* and *index.rst*).

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
