#! /usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
import random
from assignment_1.srv import *
from assignment_1.msg import OICommandResult
from assignment_1.msg import SMCommandResult
from armor_api.armor_client import ArmorClient

class PlannerAction(object):
    def __init__(self):
        self._as = SimpleActionServer('action_planner',
                                      assignment_1.msg.SMCommandAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self.OI_client = actionlib.SimpleActionClient('OntologyInterface',
                                                 assignment_1.msg.OICommandAction)
        self.OI_client.wait_for_server() #goes here?

        try:
            self.planning_time = rospy.get_param("planning_time")
        except:
            print('WARNING: No params on server, using default ones')
            self.planning_time = 5

        self._as.start()
        print('planner ok')

    def execute_callback(self, goal):
        result = SMCommandResult()
        if(goal.command == 'recharge'):
            result.res = self.recharge_plan()
            self._as.set_succeeded(result)
        elif(goal.command == 'monitoring'):
            result.res = self.monitoring_plan()
            self._as.set_succeeded(result)
        else:
            #error
            self._as.set_aborted()



    def recharge_plan(self):
        goal = assignment_1.msg.OICommandGoal()
        goal.command = 'recharge'
        self.OI_client.send_goal(goal)
        self.OI_client.wait_for_result()
        target = self.OI_client.get_result().res
        #PLAN HOW TO REACH TARGET
        time.sleep(self.planning_time)
        return target

    def monitoring_plan(self):
        target = self.choose_next_location()
        #PLAN HOW TO REACH TARGET
        time.sleep(self.planning_time)
        return target

    def choose_next_location(self):
        #GET URGENT ROOMS FROM THE ONTOLOGY
        get_urgent_locations = assignment_1.msg.OICommandGoal()
        get_urgent_locations.command = 'urgent'
        self.OI_client.send_goal(get_urgent_locations)
        result = OICommandResult()
        self.OI_client.wait_for_result()
        urgent_locations = self.OI_client.get_result().list

        #print(urgent_locations)

        try:
            #urgent_rooms = [value for value in urgent_locations if value in self.rooms]
            urgent_rooms = [value for value in urgent_locations if value not in self.corridors]
        except:
            get_rooms = assignment_1.msg.OICommandGoal()
            get_rooms.command = 'rooms'
            self.OI_client.send_goal(get_rooms)
            result = OICommandResult()
            self.OI_client.wait_for_result()
            self.rooms = self.OI_client.get_result().list

            get_corridors = assignment_1.msg.OICommandGoal()
            get_corridors.command = 'corridors'
            self.OI_client.send_goal(get_corridors)
            result = OICommandResult()
            self.OI_client.wait_for_result()
            self.corridors = self.OI_client.get_result().list

            urgent_rooms = [value for value in urgent_locations if value not in self.corridors]

        print('')
        print('Urgent rooms:')
        print(urgent_rooms)

        #GET REACHABLE LOCATIONS FROM THE ONTOLOGY
        get_reachable = assignment_1.msg.OICommandGoal()
        get_reachable.command = 'canReach'
        self.OI_client.send_goal(get_reachable)
        result = OICommandResult()
        self.OI_client.wait_for_result()
        reachable = self.OI_client.get_result().list
        if(not reachable):
            return #error

        #CHOOSE NEXT LOCATION FOLLOWING THE PRIORITY ALGORITHM
        #list of reachable urgent rooms
        target = [value for value in urgent_rooms if value in reachable]
        # try:
        #     target = [value for value in target if value not in self.corridors]
        # except:
        #     get_corridors = assignment_1.msg.OICommandGoal()
        #     get_corridors.command = 'corridors'
        #     self.OI_client.send_goal(get_corridors)
        #     result = OICommandResult()
        #     self.OI_client.wait_for_result()
        #     self.corridors = self.OI_client.get_result().list
        #     target = [value for value in target if value not in self.corridors]

        print('Urgent and reachable rooms:')
        print(target)

        if target:  #if not empty
            #take a random location from the list
            return random.choice(target)

        #take the reachable corridors
        target = [value for value in self.corridors if value in reachable]

        print('Reachable corridors:')
        print(target)

        if target: #if not empty
            #take a random location from the list
            return random.choice(target)

        #take a random location from the reachable locations
        return random.choice(reachable)


if __name__ == '__main__':
    rospy.init_node('planner')
    server = PlannerAction()
    rospy.spin()
