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
from std_msgs.msg import String, Float64
from control_msgs.msg import JointControllerState

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

        #self.scanning_time = rospy.get_param("scanning_time", 5)
        # Publisher:
        self.set_camera_joint = rospy.Publisher('/robot_camera_laser/joint_camera_position_controller/command', Float64, queue_size=10)
        self.set_joint1 = rospy.Publisher('/robot_camera_laser/joint1_position_controller/command', Float64, queue_size=10)
        self.set_joint2 = rospy.Publisher('/robot_camera_laser/joint2_position_controller/command', Float64, queue_size=10)

        self.locations_info = rospy.ServiceProxy('room_info', RoomInformation)
        rospy.Subscriber("/marker_publisher_mod/id", String, self.id_detected)
        self.ids = []

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
        #SCAN AND RETRIEVED THE IDs THEN PASS THEM TO ONTOLOGY
        self.ids = []
        #ids = ['11', '12', '13', '14', '15', '16', '17']
        #print(ids)


        while(len(self.ids) != 7):
            rotation_1 = -3
            rotation_2 = 0.5 #0.40
            rotation_3 = -0.8 #-0.5

            while rotation_1 <= 6.28:
                self.set_joint1.publish(rotation_1)
                self.set_joint2.publish(rotation_2)
                self.set_camera_joint.publish(rotation_3)

                self.wait_for_joint(1)

                rotation_1 = rotation_1 + 0.5
                print(self.ids)

            rotation_1 = 0
            rotation_2 = 0.40
            rotation_3 = 0.5

            while rotation_1 <= 6.28:
                self.set_joint1.publish(rotation_1)
                self.set_joint2.publish(rotation_2)
                self.set_camera_joint.publish(rotation_3)

                self.wait_for_joint(1)

                rotation_1 = rotation_1 + 0.5
                print(self.ids)

            print(self.ids)

        self.set_camera_joint.publish(0)
        self.set_joint1.publish(0)
        self.set_joint2.publish(0)
        self.wait_for_joint(1)
        self.wait_for_joint(2)
        self.wait_for_joint(3)
        # send ids to OI interface
        OI_cmd = assignment_1.msg.OICommandGoal()
        OI_cmd.command = 'load_map'
        OI_cmd.ids = self.ids

        self.OI_client.send_goal(OI_cmd)
        result = OICommandResult()
        self.OI_client.wait_for_result()
        return self.OI_client.get_result().res

    def id_detected(self, marker):
        id = int(marker.data)
        if self.ids.count(id) == 0:
            res = self.locations_info(id)
            if res.room != "no room associated with this marker id":
                self.ids.append(id)

    def wait_for_joint(self, joint):
        err = 1
        while(err > 0.05):
            if(joint == 1):
                    res = rospy.wait_for_message('/robot_camera_laser/joint1_position_controller/state',JointControllerState)
            elif(joint == 2):
                    res = rospy.wait_for_message('/robot_camera_laser/joint2_position_controller/state',JointControllerState)
            elif(joint == 3):
                    res = rospy.wait_for_message('/robot_camera_laser/joint_camera_position_controller/state',JointControllerState)
            err = res.error



if __name__ == '__main__':
    rospy.init_node('scanner')
    server = ScannerAction()
    rospy.spin()
