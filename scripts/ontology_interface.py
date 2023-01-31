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

        - it initializes the action server

        - it initializes the room_info service

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

        # params setup
        self.ontology_path = rospy.get_param("O_path", "$(find assignment_1)/ontology/map1.owl")
        self.ontology_iri = rospy.get_param("O_IRI", "http://bnc/exp-rob-lab/2022-23")
        self.recharge_room = rospy.get_param("charging_station_in", "E")
        self.ontology_reasoner = rospy.get_param("ontology_reasoner", "PELLET")
        self.armor_client_id = rospy.get_param("armor_client_id", "client")
        self.armor_reference_name = rospy.get_param("armor_reference_name", "ref")
        self.urgencyThreshold =  str(rospy.get_param("urgency_threshold", str(7)))
        self.save_ontology = rospy.get_param("save_ontology", False)
        self.saved_ontology_path = rospy.get_param("O_save_path", "/home//mapFull.owl")

        # define the armor client
        self.armor_client = ArmorClient(self.armor_client_id, self.armor_reference_name)

        self.map_loaded = False

        rospy.wait_for_service('/armor_interface_srv', 30)

        self._as.start()
        rospy.loginfo('OI ok')

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
        Popolates the ontology according to the markers ids and save the names of the robot and all the locations, rooms and corridors
        """

        #LOAD THE ONTOLOGY
        self.armor_client.utils.load_ref_from_file(self.ontology_path, self.ontology_iri, buffered_manipulation=False, reasoner=self.ontology_reasoner, buffered_reasoner=False, mounted=False)
        # look for the robot name
        self.robot = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROBOT']))[0]
        # assuming the ontology can already have some elements saved other than the robot itself
        doors = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['DOOR']))
        locations = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['LOCATION']))

        rospy.loginfo('Adding locations:')
        for id in ids:
            res = self.locations_info(int(id))
            room = res.room;
            position_x = str(res.x);
            position_y = str(res.y);
            # if the room does not exist, add it
            if(locations.count(room) == 0):
                self.armor_client.call('ADD','IND','CLASS',[room, 'LOCATION'])
                locations.append(room)
            # add the coordinates
            self.armor_client.call('ADD','DATAPROP','IND',['position_x', room, 'Float', position_x])
            self.armor_client.call('ADD','DATAPROP','IND',['position_y', room, 'Float', position_y])
            # last visitedAt = 0
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

                # if the location does not exist, add it
                if(locations.count(conn_location) == 0):
                    self.armor_client.call('ADD','IND','CLASS',[conn_location, 'LOCATION'])
                    locations.append(conn_location)

        # here I should have the complete ontology, 7 ids for 7 markers
        self.armor_client.call('DISJOINT','IND','CLASS',['LOCATION'])
        self.armor_client.call('DISJOINT','IND','CLASS',['DOOR'])
        self.update_timestamp()

        #SAVE THE CONSTANTS LISTS
        self.rooms = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['ROOM']))
        self.corridors = self.clean_response_list(self.armor_client.call('QUERY','IND','CLASS',['CORRIDOR']))
        rospy.loginfo('Locations:')
        rospy.loginfo(self.rooms)
        rospy.loginfo('of which, corridors:')
        rospy.loginfo(self.corridors)

        # At first the robot is in the recharging room
        self.armor_client.call('ADD','OBJECTPROP','IND',['isIn', self.robot, self.recharge_room])
        # Update its visitedAt
        old = str(0)
        new = str(round(time.time()))
        self.armor_client.call('REPLACE','DATAPROP','IND',['visitedAt',self.recharge_room,'Long',new,old])

        # update urgencyThreshold
        old_uT = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['urgencyThreshold',self.robot]))[0]
        self.clean_response_list(self.armor_client.call('REPLACE','DATAPROP','IND',['urgencyThreshold',self.robot,'Long',self.urgencyThreshold, old_uT]))

        self.update_timestamp()

        if self.save_ontology:
            rospy.loginfo('Saving the popolated ontology')
            self.armor_client.call('SAVE', 'INFERENCE', '', [self.saved_ontology_path])

        self.map_loaded = True
        return 'map_loaded'

    def choose_next_location(self):
        """
        Choose the next target following the following algorithm

        - choose an urgent room (randomly) if reachable

        - else, choose a corridor (randomly)

        - else, choose any reachable room (randomly)
        """
        print(' ')

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
        targets = [value for value in urgent_rooms if value in reachable]
        #if not empty
        if targets:
            #take a random location from the list
            next = random.choice(targets)
            earliest_time = round(time.time())
            # look for the location not visited for the longest time
            for target in targets:
                visited_time = int(self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['visitedAt',next]))[0])
                #print(target+' seen at '+str(visited_time))
                if earliest_time > visited_time:
                    earliest_time = visited_time
                    next = target
            rospy.loginfo('Among urgents ')
            rospy.loginfo(targets)
            rospy.loginfo('I am chosing '+next)
            if(len(targets)>1):
                rospy.loginfo(' since it is the reachable location not visited for the longest time')
        else:
            #take the reachable corridors
            targets = [value for value in self.corridors if value in reachable]
            #if not empty
            if targets:
                higher_n_urgents = 0
                next = random.choice(targets)
                for target in targets:
                    can_reach_from = self.clean_response_list(self.armor_client.call('QUERY','OBJECTPROP','IND',['connectedTo',target]))
                    reachable_urgents =  [value for value in urgent_rooms if value in can_reach_from]
                    if len(reachable_urgents) > higher_n_urgents:
                        higher_n_urgents = len(reachable_urgents)
                        next = target

                rospy.loginfo('Among corridors ')
                rospy.loginfo(targets)
                rospy.loginfo('I am chosing '+next)
                if(len(targets)>1 and higher_n_urgents>0):
                    rospy.loginfo('since, from there, the robot can reach '+str(higher_n_urgents)+' urgent rooms')

            else:
                #take a random location from the reachable locations
                next = random.choice(reachable)

        position_x = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['position_x',next]))
        position_y = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['position_y',next]))
        return [next, position_x, position_y]


    def set_location(self, to):
        """
        Set a new robot location according to the input
        It replaces the robot's isIn value and the new location's visitedAt value with the new ones
        """

        old_uT = self.clean_response_list(self.armor_client.call('QUERY','DATAPROP','IND',['urgencyThreshold',self.robot]))[0]

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
