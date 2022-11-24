#!/usr/bin/env python

import roslib
import rospy
import actionlib
import time
import random
import assignment_1.msg
from armor_api.armor_client import ArmorClient
from assignment_1.srv import Command
import assignment_1

try:
    recharge_room = rospy.get_param("/charging_station_in")
    ontology_reasoner = rospy.get_param("ontology_reasoner")
    armor_client_id = rospy.get_param("armor_client_id")
    armor_reference_name = rospy.get_param("armor_reference_name")
    wait_time = rospy.get_param("monitoring_duration")                  #!!!!! maybe goes inside controller
except:
    print('WARNING: No params on server, using default ones')
    recharge_room = "E"
    ontology_reasoner = "PELLET"
    armor_client_id = "client"
    armor_reference_name = "ref"
    wait_time = 5

map_loaded = 0

def clean_response_list(res):
    #take the actual list
    list = res.queried_objects
    #remove the IRI
    list = [s.replace('<'+iri+'#', '') for s in list]
    list = [s.replace('>', '') for s in list]
    #remove the timestamp associated characters
    list = [s.replace('"', '') for s in list]
    list = [s.replace('^^xsd:long', '') for s in list]
    return list

def move_robot(to):
    #old_location = clean_response_list(client.call('QUERY','OBJECTPROP','IND',['isIn',robot]))[0]
    old_location = armor_client.call('QUERY','OBJECTPROP','IND',['isIn',robot])
    old_location = clean_response_list(old_location)[0]
    armor_client.call('REPLACE','OBJECTPROP','IND',['isIn', robot,  to, old_location])
    res = armor_client.call('QUERY','DATAPROP','IND',['visitedAt',to])
    old = str(clean_response_list(res)[0])
    new = str(round(time.time()))
    armor_client.call('REPLACE','DATAPROP','IND',['visitedAt',to,'Long',new,old])
    #print('going to '+ to)
    update_timestamp()

def update_timestamp():
    armor_client.call('REASON','','',[''])
    res = armor_client.call('QUERY','DATAPROP','IND',['now',robot])
    old = str(clean_response_list(res)[0])
    new = str(round(time.time()))
    armor_client.call('REPLACE','DATAPROP','IND',['now',robot,'Long',new,old])

def choose_destination():
    #should I update the timestamp?
    urgent_locations = clean_response_list(armor_client.call('QUERY','IND','CLASS',['URGENT']))
    urgent_rooms = [value for value in urgent_locations if value in rooms]
    print('Urgent now:')
    print(urgent_rooms)
    print(' ')
    reachable = clean_response_list(res = armor_client.call('QUERY','OBJECTPROP','IND',['canReach',robot]))

    target = [value for value in urgent_rooms if value in reachable]

    if target:
        target = random.choice(target)
    else:
        target = [value for value in corridors if value in reachable]
        if target:
            target = random.choice(target)
        else:
            target = random.choice(reachable)
            if not target:
                #not possible
                return

    #print(target)
    move_robot(target)
    return target

def load_map():
    global iri, armor_client, robot, rooms, corridors
    # calling the scanner to load the environment information
    action_client = actionlib.SimpleActionClient('scanner',
                                                 assignment_1.msg.ScannerAction)
    action_client.wait_for_server()
    goal = assignment_1.msg.ScannerGoal()
    action_client.send_goal(goal)
    action_client.wait_for_result()
    #    print(action_client.get_state())
    path = action_client.get_result().ontology_path
    iri = action_client.get_result().ontology_iri

    # loading the ontology representing the environment
    armor_client = ArmorClient(armor_client_id, armor_reference_name)
    armor_client.utils.load_ref_from_file(path, iri, buffered_manipulation=False, reasoner='PELLET', buffered_reasoner=False, mounted=False)

    robot = clean_response_list(armor_client.call('QUERY','IND','CLASS',['ROBOT']))[0]
    rooms = clean_response_list(armor_client.call('QUERY','IND','CLASS',['ROOM']))
    corridors = clean_response_list(armor_client.call('QUERY','IND','CLASS',['CORRIDOR']))

    update_timestamp()

    return 'done'

def handle_sm_command(req):
    if(req.command == 'load_map'):
        global map_loaded
        if map_loaded:
            print('map already loaded')
            return 'done'
        print('loading map..')
        map_loaded = 1
        return load_map()

    elif(req.command == 'move_to'):
        #print('choosing next location..')
        return choose_destination()

    elif(req.command == 'recharge'):
        print('going to recharge..')
        move_robot(recharge_room)
        return 'done'


def main():
    rospy.init_node('test')

    s = rospy.Service('command', Command, handle_sm_command)

    rospy.spin()





    # res = client.call('QUERY','OBJECTPROP','IND',['isIn',robot])
    # print('im in '+ clean_response_list(res)[0])
    #
    # choose_destination()
    #
    #
    # res = client.call('QUERY','OBJECTPROP','IND',['isIn',robot])
    # print('im in '+ clean_response_list(res)[0])
    #client.query.check_ind_exists('http://bnc/exp-rob-lab/2022-23#Robot1'))
    #print(client.query.ind_b2_class('LOCATION'))
    # res = client.call('QUERY','IND','CLASS',['URGENT'])
    # print(clean_response_list(res))
    #res = client.call('QUERY','CLASS','CLASS',['LOCATION'])
    #res = client.call('QUERY','OBJECTPROP','IND',['connectedTo','C1'])
    #res = client.call('QUERY','OBJECTPROP','IND',['canReach',robot])
    #res = client.call('QUERY','OBJECTPROP','IND',['isIn',robot])
    #res = client.call('QUERY','DATAPROP','IND',['visitedAt','R2'])



    # res = client.call('QUERY','DATAPROP','IND',['now',robot])
    # print(clean_response_list(res))
    #
    # res = client.call('QUERY','OBJECTPROP','IND',['isIn',robot])
    # print('im in '+ clean_response_list(res)[0])
    #
    # res = client.call('QUERY','OBJECTPROP','IND',['canReach',robot])
    # print(clean_response_list(res))
    #
    # move_robot( 'C1', 'R1')
    # print('done')
    #
    # res = client.call('QUERY','DATAPROP','IND',['now',robot])
    # print(clean_response_list(res))
    #
    # res = client.call('QUERY','OBJECTPROP','IND',['isIn',robot])
    # print(clean_response_list(res))
    #
    # res = client.call('QUERY','OBJECTPROP','IND',['canReach',robot])
    # print(clean_response_list(res))
    # res = client.call('QUERY','DATAPROP','IND',['urgencyThreshold',robot])
    # update_timestamp()
    # res = client.call('QUERY','DATAPROP','IND',['visitedAt','C1'])
    # print(clean_response_list(res))
    #
    # res = client.call('QUERY','IND','CLASS',['URGENT'])
    # print(clean_response_list(res))


    #print(clean_response_list_from_iri(res))

    # Wait for ctrl-c to stop the application
    #rospy.spin()










if __name__ == '__main__':
  main()
