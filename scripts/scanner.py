#! /usr/bin/env python

"""
.. module::scanner
    :platform: Unix
    :synopsis: Python module for the state machine
.. moduleauthor:: Samuele Depalo depalo.samuele@gmail.com

ROS node for controlling the camera arm of the proposed robot

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
from assignment_1.msg import OICommandResult, ScannerResult, OICommandGoal
from std_msgs.msg import String, Float64
from control_msgs.msg import JointControllerState

class ScannerAction(object):
    """
    Class implementing the scanner action server
    """

    def __init__(self):
        """
        Initialize the server and a client for the ontology interface node
        """
        self._as = SimpleActionServer('action_scanner',
                                      assignment_1.msg.ScannerAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self.OI_client = actionlib.SimpleActionClient('OntologyInterface',
                                                 assignment_1.msg.OICommandAction)
        self.OI_client.wait_for_server()

        self.n_ids = rospy.get_param("n_ids", 7)
        # Publisher:
        self.set_camera_joint = rospy.Publisher('joint_camera_position_controller/command', Float64, queue_size=10)
        self.set_joint1 = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)
        self.set_joint2 = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=10)

        self.locations_info = rospy.ServiceProxy('room_info', RoomInformation)
        rospy.Subscriber("/marker_publisher_mod/id", String, self.id_detected)
        self.ids = []

        self._as.start()
        rospy.loginfo('scanner ok')

    def execute_callback(self, goal):
        """
        When receives a goal, if it makes sense, call the corresponding function
        """
        print(' ')
        result = ScannerResult()
        if(goal.command == 'load_map'):
            result.res = self.load_map()
            self._as.set_succeeded(result)
        elif(goal.command == 'scan_room'):
            result.res = self.scan()
            self._as.set_succeeded(result)
        else:
            #error
            self._as.set_aborted()

    def scan(self):
        """
        Rotates the camera of 360 degrees
        """
        rospy.loginfo('Scanning..')
        self.set_joint1.publish(0)
        rospy.sleep(0.2)

        self.set_joint1.publish(1.57)
        rospy.sleep(0.2)

        self.set_joint1.publish(3.14)
        rospy.sleep(0.2)

        self.set_joint1.publish(4.71)
        rospy.sleep(0.2)

        self.set_joint1.publish(6.28)
        rospy.sleep(0.2)
        rospy.loginfo('Done!')
        return 'done'


    def load_map(self):
        """
        Scan the markers and send the ids to the ontology_interface in order to load the map
        """
        rospy.loginfo('Looking for markers, below the list while updating:')
        #SCAN AND RETRIEVED THE IDs THEN PASS THEM TO ONTOLOGY
        self.ids = []
        self.ids = [11, 12, 13, 14, 15, 16, 17]

        while(len(self.ids) != self.n_ids):
            rotation_1 = -3
            rotation_2 = 0.5 #0.40
            rotation_3 = -0.8 #-0.5

            # 360 degrees with an arm configuration
            while rotation_1 <= 6.28:
                self.set_joint1.publish(rotation_1)
                self.set_joint2.publish(rotation_2)
                self.set_camera_joint.publish(rotation_3)

                self.wait_for_joint(1)

                rotation_1 = rotation_1 + 0.5
                rospy.loginfo(self.ids)

            rotation_1 = 0
            rotation_2 = 0.40
            rotation_3 = 0.5

            # 360 degrees with another arm configuration
            while rotation_1 <= 6.28:
                self.set_joint1.publish(rotation_1)
                self.set_joint2.publish(rotation_2)
                self.set_camera_joint.publish(rotation_3)

                self.wait_for_joint(1)

                rotation_1 = rotation_1 + 0.5
                rospy.loginfo(self.ids)

            rospy.loginfo(self.ids)

        # arm back to standard config
        self.set_camera_joint.publish(0)
        self.set_joint1.publish(0)
        self.set_joint2.publish(0)
        self.wait_for_joint(1)
        self.wait_for_joint(2)
        self.wait_for_joint(3)
        # send ids to OI interface
        OI_cmd = OICommandGoal()
        OI_cmd.command = 'load_map'
        OI_cmd.ids = self.ids
        self.OI_client.send_goal(OI_cmd)
        result = OICommandResult()
        self.OI_client.wait_for_result()
        return self.OI_client.get_result().res

    def id_detected(self, marker):
        """
        Get the id and checks if it's valid
        """
        id = int(marker.data)
        if self.ids.count(id) == 0:
            res = self.locations_info(id)
            if res.room != "no room associated with this marker id":
                self.ids.append(id)

    def wait_for_joint(self, joint):
        """
        Wait until the joint reaches the desired configuration
        """
        err = 1
        while(err > 0.05):
            if(joint == 1):
                    res = rospy.wait_for_message('/joint1_position_controller/state',JointControllerState)
            elif(joint == 2):
                    res = rospy.wait_for_message('/joint2_position_controller/state',JointControllerState)
            elif(joint == 3):
                    res = rospy.wait_for_message('/joint_camera_position_controller/state',JointControllerState)
            err = res.error

if __name__ == '__main__':
    rospy.init_node('scanner')
    server = ScannerAction()
    rospy.spin()
