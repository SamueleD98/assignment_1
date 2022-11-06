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
from armor_api.armor_client import ArmorClient

class OntologyInterface(object):
    def __init__(self):
        self._as = SimpleActionServer('OntologyInterface',
                                      assignment_1.msg.OICommandAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self.map_loaded = False
        try:
            self.ontology_path = rospy.get_param("O_path")
            self.ontology_iri = rospy.get_param("O_IRI")
            self.recharge_room = rospy.get_param("charging_station_in")
            self.ontology_reasoner = rospy.get_param("ontology_reasoner")
            self.armor_client_id = rospy.get_param("armor_client_id")
            self.armor_reference_name = rospy.get_param("armor_reference_name")
        except:
            print('WARNING: No params on server, using default ones')
            self.ontology_path = "/home/samuele/experimental_lab__ws/src/assignment_1/ontology/map1.owl"
            self.ontology_iri = "http://bnc/exp-rob-lab/2022-23"
            self.recharge_room = "E"
            self.ontology_reasoner = "PELLET"
            self.armor_client_id = "client"
            self.armor_reference_name = "ref"

        self.armor_client = ArmorClient(self.armor_client_id, self.armor_reference_name)

        rospy.wait_for_service('/armor_interface_srv', 30)


        self._as.start()
        print('OI ok')

    def execute_callback(self, goal):
        result = OICommandResult()
        #print(goal)
        if(goal.command == 'load_map'):
            result.res = self.load_map()
            self.map_loaded = True
            self._as.set_succeeded(result)
        # elif(not self.map_loaded):
        #     #error
        #     self._as.set_aborted()
        #     return
        elif(goal.command == 'move_to'):
            target = goal.location
            result.res = self.set_location(target)
            self.update_timestamp()
            self._as.set_succeeded(result)
        elif(goal.command == 'recharge'):
            result.res = self.recharge_room
            self._as.set_succeeded(result)
        elif(goal.command == 'urgent'):
            self.update_timestamp()
            result.list = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['URGENT']))
            self._as.set_succeeded(result)
        elif(goal.command == 'canReach'):
            result.list = self.clean_response_list(self.armor_client.call('QUERY','OBJECTPROP','IND',['canReach',self.robot]))
            self._as.set_succeeded(result)
        elif(goal.command == 'corridors'):
            result.list = self.corridors
            self._as.set_succeeded(result)
        elif(goal.command == 'rooms'):
            result.list = self.rooms
            self._as.set_succeeded(result)
        elif(goal.command == 'locations'):
            result.list = self.locations
            self._as.set_succeeded(result)

        else:
            #error
            self._as.set_aborted()

        return



    def load_map(self):

        #LOAD THE ONTOLOGY
        self.armor_client.utils.load_ref_from_file(self.ontology_path, self.ontology_iri, buffered_manipulation=False, reasoner=self.ontology_reasoner, buffered_reasoner=False, mounted=False)

        #SAVE THE CONSTANTS LISTS
        self.robot = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROBOT']))[0]
        self.locations = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['LOCATION']))
        self.rooms = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROOM']))
        self.corridors = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['CORRIDOR']))

        self.update_timestamp()

        return 'map_loaded'

    def set_location(self, to):
        #old_location = clean_response_list(client.call('QUERY','OBJECTPROP','IND',['isIn',robot]))[0]
        old_location = self.armor_client.call('QUERY','OBJECTPROP','IND',['isIn', self.robot])
        old_location = self.clean_response_list(old_location)[0]
        self.armor_client.call('REPLACE','OBJECTPROP','IND',['isIn', self.robot,  to, old_location])
        res = self.armor_client.call('QUERY','DATAPROP','IND',['visitedAt',to])
        old = str(self.clean_response_list(res)[0])
        new = str(round(time.time()))
        self.armor_client.call('REPLACE','DATAPROP','IND',['visitedAt',to,'Long',new,old])
        return 'done'



    def update_timestamp(self):
        self.armor_client.call('REASON','','',[''])
        res = self.armor_client.call('QUERY','DATAPROP','IND',['now',self.robot])
        old = str(self.clean_response_list(res)[0])
        new = str(round(time.time()))
        self.armor_client.call('REPLACE','DATAPROP','IND',['now',self.robot,'Long',new,old])

    def clean_response_list(self, res):
        #take the actual list
        list = res.queried_objects
        #remove the IRI
        list = [s.replace('<'+self.ontology_iri+'#', '') for s in list]
        list = [s.replace('>', '') for s in list]
        #remove the timestamp associated characters
        list = [s.replace('"', '') for s in list]
        list = [s.replace('^^xsd:long', '') for s in list]
        return list

if __name__ == '__main__':
    rospy.init_node('ontology_interface')
    server = OntologyInterface()
    rospy.spin()
