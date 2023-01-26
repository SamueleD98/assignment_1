#! /usr/bin/env python

"""
.. module::state_machine
    :platform: Unix
    :synopsis: Python module for the state machine
.. moduleauthor:: Samuele Depalo depalo.samuele@gmail.com

ROS node for implementing a state machine which sets the desired robot's behaviour

Action client(s):

- action_scanner

- action_planner

- action_controller

- OntologyInterface


"""

import roslib
import rospy
from smach import StateMachine, State, Concurrence
from smach_ros import MonitorState, IntrospectionServer
import time
import random
import actionlib
from assignment_1.msg import OICommandGoal,ScannerAction, ScannerGoal, PlanAction, PlanGoal, ControlAction, ControlGoal, Point#, PlanResult ControlResult
from assignment_1.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#import assignment_1.msg

#import assignment_1
from std_msgs.msg import String, Bool

battery_is_low = False

class Mapping(State):
    """
    Class implementing the Mapping state of the state machine
    """
    def __init__(self):
        State.__init__(self, outcomes=['map_loaded'])
        self.scanning_time = rospy.get_param("scanning_time", 5)

    def execute(self, userdata):
        """
        Mapping state callback
        Sends a goal "load_map" to the scanner node and wait for it to end
        """
        print(' ')
        rospy.loginfo('Executing state MAPPING')

        #time.sleep(self.scanning_time)

        goal = assignment_1.msg.ScannerGoal()
        goal.command = 'load_map'
        scanner_client.send_goal(goal)
        scanner_client.wait_for_result()

        return 'map_loaded'

class Move(State):
    """
    Class implementing the Move state of the state machine
    Manages the motion to a location:

    - asks the Ontology Interface for the target.
    - asks the Planner for a plan.
    - asks the Controller to control the motion
    - asks the Ontology Interface to set the new robot position in the ontology

    """
    def __init__(self, type):
        State.__init__(self, outcomes=['done','preempted'], )
        self._type = type   # distinguish a recharge move from a monitor move
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()

    def execute(self, userdata):
        print(' ')

        # get the next location from the ontolgy interface
        get_target = OICommandGoal()
        if self._type == 'recharge':
            get_target.command = 'recharge_room'
        elif self._type == 'monitor':
            get_target.command = 'next_room'

        # wait for the result and check if the state is preempted
        OI_client.send_goal(get_target)
        while(OI_client.get_state() != 3):
            if self.preempt_requested():
                OI_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        OI_client.wait_for_result()

        # next location
        result = OI_client.get_result()
        target = result.res
        pos_x = float(result.x)
        pos_y = float(result.y)
        rospy.loginfo('Going to '+ target)

        #PLANNER
        #actually there exist a make_plan srv for move_base
        get_plan = PlanGoal()
        get_plan.target = Point(x=pos_x,
                                y=pos_y)

        # wait for the result and check if the state is preempted
        planner_client.send_goal(get_plan)
        while(planner_client.get_state() != 3):
            if self.preempt_requested():
                planner_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        planner_client.wait_for_result()

        # plan
        plan = planner_client.get_result()

        #CONTROLLER
        #goal = ControlGoal()
        #goal.via_points = plan.via_points

        # wait for the result and check if the state is preempted
        #controller_client.send_goal(goal)
        #while(controller_client.get_state() != 3):
        #    if self.preempt_requested():
        #        controller_client.cancel_all_goals()
        #        self.service_preempt()
        #        return 'preempted'
        #controller_client.wait_for_result()

        #MOVE_BASE
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pos_x
        goal.target_pose.pose.position.x = pos_y
        goal.target_pose.pose.orientation.w= 1.0;

        self.move_base_client.send_goal(goal)
        while(self.move_base_client.get_state() != 3):
            if self.preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        self.move_base_client.wait_for_result()

        # update the robot position in the ontology
        set_new_room = OICommandGoal()
        set_new_room.command = 'move_to'
        set_new_room.location = target

        # wait for the result and check if the state is preempted
        OI_client.send_goal(set_new_room)
        while(OI_client.get_state() != 3):
            if self.preempt_requested():
                OI_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        OI_client.wait_for_result()

        rospy.loginfo('Reached '+ target)
        return 'done'



class Monitor(State):
    """
    Class implementing the Monitor state of the state machine
    Waits for the given time while checking if the state is preempted
    """
    def __init__(self):
        State.__init__(self, outcomes=['done','preempted'] )
        self.monitoring_time = rospy.get_param("monitoring_time", 10)

    def execute(self, userdata):
        rospy.loginfo('Monitoring the environment')

        # this should be replaced with the actual code for exploring the environment
        for x in range(self.monitoring_time):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(1)

        return 'done'

class Recharge(State):
    """
    Class implementing the Recharge state of the state machine
    Waits for the given time while checking if the state is preempted
    """
    def __init__(self):
        State.__init__(self, outcomes=['done'] )
        self.recharging_time = rospy.get_param("recharging_time", 10)

    def execute(self, userdata):
        # print(' ')
        rospy.loginfo('Recharging')

        # this should be replaced with the actual code for recharging the robot
        for x in range(self.recharging_time):
            if self.preempt_requested():
                self.service_preempt()
                return 'done'
            time.sleep(1)

        return 'done'


# gets called when ANY child state terminates
def child_term_cb(outcome_map):
    """
    Function called when an inner state terminate
    It just terminates all the inner states
    """
    # terminate ALL the child states
    return True

# gets called when ALL child states are terminated
def out_cb_monitoring(outcome_map):
    """
    Function called when all the MONITORING-state's inner states terminate
    It chose the outcome of the main state

    Returns:
        'battery_low' if the inner state machine EXECUTE  has been preempted

        'monitoring_done' if EXECUTE ended the execution
    """
    if outcome_map['EXECUTE'] == 'preempted':
        return 'battery_low'
    else:
        return 'monitoring_done'
def out_cb_recharge(outcome_map):
    """
    Function called when all the RECHARGING-state's inner states terminate
    It sets the outcome of the main state

    Returns:
        'recharge_done'
    """
    global battery_is_low
    battery_is_low = False  # update the boolean, the battery is high
    return 'recharge_done'

def monitor_cb(ud, msg):
    """
    Function called when a message is published in the battery_status topic
    Decides if preempting the states based on the message
    """
    global battery_is_low
    rospy.loginfo('BATTERY STATUS UPDATE: is battery low? --> ' + str(msg.data))

    # if the message doesn't bring an update
    if battery_is_low == msg.data :
        # make the execution continue
        return True
    battery_is_low = msg.data # update the value
    # stop the execution, a change in states is required
    return False


def main():
    """
    This function initializes the ROS node, the state machine and the clients for the :mod:`scanner`, :mod:`planner`, :mod:`controller` and :mod:`ontology_interface` modules.
    """
    global OI_client, scanner_client, planner_client, controller_client
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm_main = StateMachine([])

    # Open the container
    with sm_main:
        #---------- MAPPING STATE ----------
        StateMachine.add('MAPPING', Mapping(),
                               transitions={'map_loaded':'MONITORING'})

        #---------- MONITORING (CONCURRENCE) STATE ----------
        sm_monitoring = Concurrence(outcomes=['monitoring_done','battery_low'],
                                   default_outcome='monitoring_done',
                                   child_termination_cb = child_term_cb,
                                   outcome_cb = out_cb_monitoring)
        with sm_monitoring:
            Concurrence.add('CHECK_BATTERY_STATUS',
                                MonitorState("/battery_status", Bool, monitor_cb))

            sm_monitoring_exe = StateMachine(outcomes=['monitoring_done', 'preempted'])
            with sm_monitoring_exe:
                StateMachine.add('MOVE_TO_MONITOR', Move(type = 'monitor'),
                                 transitions={'done':'MONITOR',
                                              'preempted':'preempted'})
                StateMachine.add('MONITOR', Monitor(),
                                 transitions={'done':'monitoring_done',
                                              'preempted':'preempted'})
            Concurrence.add('EXECUTE',
                            sm_monitoring_exe)

        StateMachine.add('MONITORING', sm_monitoring,
                               transitions={'monitoring_done':'MONITORING',
                                            'battery_low':'RECHARGING'})

        #---------- RECHARGE (CONCURRENCE) STATE ----------
        sm_recharge = Concurrence(outcomes=['recharge_done'],
                                   default_outcome='recharge_done',
                                   child_termination_cb = child_term_cb,
                                   outcome_cb = out_cb_recharge)
        with sm_recharge:
            Concurrence.add('CHECK_BATTERY_STATUS',
                                MonitorState("/battery_status", Bool, monitor_cb))

            sm_recharging_exe = StateMachine(outcomes=['recharge_done'])
            with sm_recharging_exe:
                StateMachine.add('MOVE_TO_RECHARGE', Move(type = 'recharge'),
                                 transitions={'done':'RECHARGE',
                                              'preempted':'recharge_done'})
                StateMachine.add('RECHARGE', Recharge(),
                                 transitions={'done':'recharge_done'})

            Concurrence.add('EXECUTE',
                                sm_recharging_exe)

        StateMachine.add('RECHARGING', sm_recharge,
                               transitions={'recharge_done':'MONITORING'})


    # Create and start the introspection server for visualization
    sis = IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()


    OI_client = actionlib.SimpleActionClient('OntologyInterface',
                                             assignment_1.msg.OICommandAction)

    scanner_client = actionlib.SimpleActionClient('action_scanner',
                                             assignment_1.msg.ScannerAction)

    planner_client = actionlib.SimpleActionClient('action_planner',
                                             assignment_1.msg.PlanAction)

    controller_client = actionlib.SimpleActionClient('action_controller',
                                             assignment_1.msg.ControlAction)

    OI_client.wait_for_server()
    scanner_client.wait_for_server()
    planner_client.wait_for_server()
    controller_client.wait_for_server()


    # Execute the state machine
    sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
