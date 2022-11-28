#! /usr/bin/env python

"""
.. module::scanner
    :platform: Unix
    :synopsis: Python module for the state machine
.. moduleauthor:: Samuele Depalo depalo.samuele@gmail.com

ROS node for simulating a scanner

Action server:

- action_scanner

"""

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
from assignment_1.srv import *
from assignment_1.msg import OICommandResult
from assignment_1.msg import ScannerResult

class ScannerAction(object):
    """
    Class implementing the scanner action server
    """

    def __init__(self):
        """
        Initialise the server and a client for the ontology interface node
        """
        self._as = SimpleActionServer('action_scanner',
                                      assignment_1.msg.ScannerAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self.OI_client = actionlib.SimpleActionClient('OntologyInterface',
                                                 assignment_1.msg.OICommandAction)
        self.OI_client.wait_for_server()

        self.scanning_time = rospy.get_param("scanning_time", 5)

        self._as.start()
        print('scanner ok')

    def execute_callback(self, goal):
        """
        When receives a goal, if it makes sense, call the load_map() function
        """
        result = ScannerResult()
        if(goal.command == 'load_map'):
            result.res = self.load_map()
            self._as.set_succeeded(result)
        else:
            #error
            self._as.set_aborted()

    def load_map(self):
        """
        Send a "load_map" goal to the ontology_interface and waits for it to succeed
        """
        #HERE HOW TO LOAD THE MAP
        time.sleep(self.scanning_time)
        OI_cmd = assignment_1.msg.OICommandGoal()
        OI_cmd.command = 'load_map'
        self.OI_client.send_goal(OI_cmd)
        result = OICommandResult()
        self.OI_client.wait_for_result()
        return self.OI_client.get_result().res


if __name__ == '__main__':
    rospy.init_node('scanner')
    server = ScannerAction()
    rospy.spin()
