#! /usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
from assignment_1.srv import *
from assignment_1.msg import OICommandResult
from assignment_1.msg import SMCommandResult
from armor_api.armor_client import ArmorClient

class ControllerAction(object):
    def __init__(self):
        self._as = SimpleActionServer('action_controller',
                                      assignment_1.msg.SMCommandAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self.OI_client = actionlib.SimpleActionClient('OntologyInterface',
                                                 assignment_1.msg.OICommandAction)
        self.OI_client.wait_for_server() #goes here?

        try:
            self.monitoring_time = rospy.get_param("monitoring_time")
            self.recharging_time = rospy.get_param("recharging_time")
        except:
            print('WARNING: No params on server, using default ones')
            self.monitoring_time = 10
            self.recharging_time = 10

        self._as.start()
        print('controller ok')

    def execute_callback(self, goal):
        result = SMCommandResult()
        try:
            self.locations
        except:
            get_locations = assignment_1.msg.OICommandGoal()
            get_locations.command = 'locations'
            self.OI_client.send_goal(get_locations)
            result = OICommandResult()
            self.OI_client.wait_for_result()
            self.locations = self.OI_client.get_result().list

        if(goal.command in self.locations):
            result.res = self.control(goal.command)
            self._as.set_succeeded(result)
        elif(goal.command == 'recharge'):
            result.res = self.recharge()
            self._as.set_succeeded(result)
        else:
            #error
            self._as.set_aborted()


    def control(self, target):
        #CONTROL ROBOT UNTIL IT REACHES AND MONITOR THE TARGET
        time.sleep(self.monitoring_time)
        #UPDATE ONTOLOGY
        goal = assignment_1.msg.OICommandGoal()
        goal.command = 'move_to'
        goal.location = target
        self.OI_client.send_goal(goal)
        result = OICommandResult()
        self.OI_client.wait_for_result()
        return self.OI_client.get_result().res

    def recharge(self):
        #HOW TO RECHARGE THE BATTERY

        #WAIT UNTILE RECHARGED
        time.sleep(self.recharging_time)
        return 'done'

if __name__ == '__main__':
    rospy.init_node('controller')
    server = ControllerAction()
    rospy.spin()
