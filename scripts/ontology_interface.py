#! /usr/bin/env python

"""
.. module::ontology_interface
    :platform: Unix
    :synopsis: Python module for the ontology interface
.. moduleauthor:: Samuele Depalo depalo.samuele@gmail.com

ROS node for quering and manipulating the ontology loaded in the `ARMOR <hhttps://github.com/EmaroLab/armor>`_ server.

Action server:

- OntologyInterface


"""

import rospy
import actionlib
import random
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
from std_msgs.msg import String
from assignment_1.srv import *
from assignment_1.msg import OICommandAction, OICommandResult
from armor_api.armor_client import ArmorClient

class OntologyInterface(object):
    """
    Class implementing an interface for the ontology
    """
    def __init__(self):
        """
        Initialise the object

        - it initialises the action server

        - gets the parameters from the rosparam server

        - initialises the armor client

        - waits for the armor service to be active

        - run the action server

        """
        self._as = SimpleActionServer('OntologyInterface',
                                      OICommandAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)

        self.locations_info = rospy.ServiceProxy('room_info', RoomInformation)

        #self.map_loaded = False

        # params setup
        self.ontology_path = rospy.get_param("O_path", "/home/samuele/experimental_lab__ws/src/assignment_1/ontology/map1.owl")
        self.ontology_iri = rospy.get_param("O_IRI", "http://bnc/exp-rob-lab/2022-23")
        self.recharge_room = rospy.get_param("charging_station_in", "E")
        self.ontology_reasoner = rospy.get_param("ontology_reasoner", "PELLET")
        self.armor_client_id = rospy.get_param("armor_client_id", "client")
        self.armor_reference_name = rospy.get_param("armor_reference_name", "ref")
        self.urgencyThreshold =  str(rospy.get_param("urgency_threshold", str(7)))

        # define the armor client
        self.armor_client = ArmorClient(self.armor_client_id, self.armor_reference_name)

        self.map_loaded = False

        rospy.wait_for_service('/armor_interface_srv', 30)

        self._as.start()
        print('OI ok')

    def execute_callback(self, goal):
        """
        Action server callback
        It calls a function according to the received command and returns the function output
        """
        result = OICommandResult()

        if(goal.command == 'load_map'):     # load a map
            result.res = self.load_map(goal.ids)
            self._as.set_succeeded(result)
        elif(not self.map_loaded):
             #error
             self._as.set_aborted()
             return
        elif(goal.command == 'next_room'):  # choose the next target
            self.update_timestamp() # update the "now" value for obtaining the correct urgent locations
            data = self.choose_next_location()
            result.res = data[0]
            x = data[1]
            y = data[2]
            result.x = x[0]
            result.y = y[0]
            self._as.set_succeeded(result)
        elif(goal.command == 'move_to'):    # set a new position for the robot
            target = goal.location
            result.res = self.set_location(target)
            self.update_timestamp()
            self._as.set_succeeded(result)
        elif(goal.command == 'recharge_room'):  # return the recharge room
                result.res = self.recharge_room
                self._as.set_succeeded(result)
        else:
            #error
            self._as.set_aborted()
        return



    def load_map(self, ids):
        """
        Load the map as ontology and save the names of the robot and all the locations, rooms and corridors
        """

        #LOAD THE ONTOLOGY
        self.armor_client.utils.load_ref_from_file(self.ontology_path, self.ontology_iri, buffered_manipulation=False, reasoner=self.ontology_reasoner, buffered_reasoner=False, mounted=False)

        ###  self.armor_client.utils.mount_on_ref()

        self.robot = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROBOT']))[0]
        # if not self.robot:
        #     print('MTFCK')
        #     self.robot = 'Robot1'

        doors = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['DOOR']))
        locations = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['LOCATION']))

        print('Adding locations:')
        for id in ids:
            res = self.locations_info(int(id))

            if res.room == "no room associated with this marker id":
                return 'wrong_ids'

            room = res.room;
            print(room)
            position_x = str(res.x);
            position_y = str(res.y);
            # if the room does not exist, add it
            if(locations.count(room) == 0):
                self.armor_client.call('ADD','IND','CLASS',[room, 'LOCATION'])
                locations.append(room)
            # add the coordinates
            self.armor_client.call('ADD','DATAPROP','IND',['position_x', room, 'Float', position_x])
            self.armor_client.call('ADD','DATAPROP','IND',['position_y', room, 'Float', position_y])

            time_zero = str(0)
            self.armor_client.call('ADD','DATAPROP','IND',['visitedAt',room,'Long',time_zero])

            # rooms may have multiple connections
            for x in res.connections:
                conn_location = x.connected_to
                conn_door = x.through_door

                # if the door does not exist, add it
                if(doors.count(conn_door) == 0):
                    self.armor_client.call('ADD','IND','CLASS',[conn_door, 'DOOR'])
                    doors.append(conn_door)
                self.armor_client.call('ADD','OBJECTPROP','IND',['hasDoor', room, conn_door])

                # actually the reasoner is able to understand the following by itself
                # if the location does not exist, add
                if(locations.count(conn_location) == 0):
                    self.armor_client.call('ADD','IND','CLASS',[conn_location, 'LOCATION'])
                    locations.append(conn_location)
                #self.armor_client.call('ADD','OBJECTPROP','IND',['connectedTo', room, conn_location])


        # here I should have the complete ontology, 7 ids for 7 markers
        self.armor_client.call('DISJOINT','IND','CLASS',['LOCATION'])
        ###  self.armor_client.call('REASON','','',[''])
        self.update_timestamp()



        #SAVE THE CONSTANTS LISTS

        self.robot = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROBOT']))[0]
        self.locations = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['LOCATION']))
        self.rooms = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROOM']))
        self.corridors = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['CORRIDOR']))

        # for location in self.locations:
        #     for another_location in self.locations:
        #         if location != another_location:
        #             self.armor_client.call('ADD','OBJECTPROP','IND',['differentFrom', location, another_location])




        self.armor_client.call('ADD','OBJECTPROP','IND',['isIn', self.robot, self.recharge_room])
        old = str(0)
        new = str(round(time.time()))
        self.armor_client.call('REPLACE','DATAPROP','IND',['visitedAt',self.recharge_room,'Long',new,old])

        old_uT = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['urgencyThreshold',self.robot]))[0]

        self.clean_response_list(self.armor_client.call('REPLACE','DATAPROP','IND',['urgencyThreshold',self.robot,'Long',self.urgencyThreshold, old_uT]))



        ###  self.armor_client.utils.apply_buffered_changes()



        ###  self.armor_client.call('REASON','','',[])

        self.update_timestamp()

        ###  self.armor_client.utils.apply_buffered_changes()

        ###  self.armor_client.call('APPLY', '', '', [])


        ###  self.armor_client.call('SAVE', 'INFERENCE', '', ["/home/samuele/experimental_lab__ws/src/assignment_1/ontology/mapMODIFIED.owl"])
        print('done')
        self.map_loaded = True
        return 'map_loaded'

    def choose_next_location(self):
        """
        Choose the next target following the following algorithm

        - choose an urgent room (randomly) if reachable

        - else, choose a corridor (randomly)

        - else, choose any reachable room (randomly)
        """


        #GET URGENT ROOMS FROM THE ONTOLOGY
        urgent_locations = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['URGENT']))
        urgent_rooms = [value for value in urgent_locations if value not in self.corridors]

        #GET REACHABLE LOCATIONS FROM THE ONTOLOGY
        reachable = self.clean_response_list(self.armor_client.call('QUERY','OBJECTPROP','IND',['canReach',self.robot]))
        if not reachable:
            rospy.logerr('ERROR: ONTOLOGY RETURNED NO REACHABLE LOCATIONS')
            return

        #CHOOSE NEXT LOCATION FOLLOWING THE PRIORITY ALGORITHM
        #list of reachable urgent rooms
        target = [value for value in urgent_rooms if value in reachable]
        #print(target)
        #print(reachable)

        #if not empty
        if target:
            #take a random location from the list
            next = random.choice(target)
        #take the reachable corridors
        target = [value for value in self.corridors if value in reachable]
        #if not empty
        if target:
            #take a random location from the list
            next = random.choice(target)
        #take a random location from the reachable locations
        next = random.choice(reachable)


        # if not target:
        #     target = [value for value in self.corridors if value in reachable]
        #     print(target)
        #     if not target:
        #         next = random.choice(reachable)
        # next = random.choice(target)


        position_x = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['position_x',next]))
        position_y = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['position_y',next]))
        return [next, position_x, position_y]





    def set_location(self, to):
        """
        Set a new robot location according to the input
        It replaces the robot's isIn value and the new location's visitedAt value with the new ones
        """

        old_uT = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['urgencyThreshold',self.robot]))[0]

        #print(old_uT)
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
        """
        Call the reasoner and replace the robot's now value with a new one
        """
        #WHAT IF REASON GOES AFTER THE REPLACE ????

        res = self.armor_client.call('QUERY','DATAPROP','IND',['now',self.robot])
        old = str(self.clean_response_list(res)[0])
        new = str(round(time.time()))
        self.armor_client.call('REPLACE','DATAPROP','IND',['now',self.robot,'Long',new,old])
        self.armor_client.call('REASON','','',[''])

    def clean_response_list(self, res):
        """
        Clean an armor query's output from the IRI and some special character
        """
        #take the actual list
        list = res.queried_objects
        #remove the IRI
        list = [s.replace('<'+self.ontology_iri+'#', '') for s in list]
        list = [s.replace('>', '') for s in list]
        #remove the timestamp associated characters
        list = [s.replace('"', '') for s in list]
        list = [s.replace('^^xsd:long', '') for s in list]
        list = [s.replace('^^xsd:float', '') for s in list]
        return list

if __name__ == '__main__':
    rospy.init_node('ontology_interface')
    server = OntologyInterface()
    rospy.spin()
