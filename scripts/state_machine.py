#! /usr/bin/env python

"""
.. module:: state_machine
:platform: Unix
:synopsis: Python module for the finite state machine
.. moduleauthor:: Samuele Depalo depalo.samuele@gmail.com

ROS node for implementing a finite state machine

Action client(s):
- Scanner
- Planner
- Controller

"""

import roslib
import rospy
from smach import StateMachine, State, Concurrence
from smach_ros import MonitorState, IntrospectionServer
import time
import random
import actionlib
from assignment_1.srv import *
import assignment_1.msg
from assignment_1.msg import OICommandGoal, PlanAction, PlanGoal, PlanResult, ControlAction, ControlGoal, ControlResult, Point
import assignment_1
from std_msgs.msg import String
from std_msgs.msg import Bool

battery_is_low = False

class Mapping(State):
    """
    Class implementing the Mapping state of the finite state machine
    """
    def __init__(self):
        """
        Initialise the Mapping state.
        It sets the state outcomes.
        """
        State.__init__(self,
                             outcomes=['map_loaded'],
                             )

    def execute(self, userdata):
        """
        Mapping state callback
        """
        print(' ')
        rospy.loginfo('Executing state MAPPING')

        goal = assignment_1.msg.SMCommandGoal()
        goal.command = 'load_map'
        scanner_client.send_goal(goal)
        scanner_client.wait_for_result()

        #print(Scanner_client.get_result().res)
        time.sleep(10)
        return 'map_loaded'
        #return 'map_loaded'

class Move(State):
    def __init__(self, type):
        State.__init__(self,
                             outcomes=['done','preempted'],
                             )
        self._type = type

    def execute(self, userdata):
        print(' ')
        get_target = OICommandGoal()
        if self._type == 'recharge':
            get_target.command = 'recharge_room'
        elif self._type == 'monitor':
            get_target.command = 'next_room'
        OI_client.send_goal(get_target)

        while(OI_client.get_state() != 3):
            if self.preempt_requested():
                OI_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        OI_client.wait_for_result()

        target = OI_client.get_result().res

        rospy.loginfo('Going to '+ target)

        #HERE CALL PLANNER
        get_plan = PlanGoal()
        get_plan.target = Point(x=random.uniform(0, 100),
                                y=random.uniform(0, 100))
        planner_client.send_goal(get_plan)
        while(planner_client.get_state() != 3):
            if self.preempt_requested():
                planner_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        planner_client.wait_for_result()


        plan = planner_client.get_result()

        #HERE CALL CONTROLLER
        goal = ControlGoal()
        goal.via_points = plan.via_points
        controller_client.send_goal(goal)
        while(controller_client.get_state() != 3):
            if self.preempt_requested():
                controller_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
        controller_client.wait_for_result()

        set_new_room = OICommandGoal()
        set_new_room.command = 'move_to'
        set_new_room.location = target
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
    def __init__(self):
        State.__init__(self,
                             outcomes=['done'],
                             )
        self.monitoring_time = rospy.get_param("monitoring_time", 10)

    def execute(self, userdata):
        # print(' ')
        rospy.loginfo('Monitoring the environment')
        time.sleep(self.monitoring_time)
        return 'done'

class Recharge(State):
    def __init__(self):
        State.__init__(self,
                             outcomes=['done'],
                             )
        self.recharging_time = rospy.get_param("recharging_time", 10)
    def execute(self, userdata):
        # print(' ')
        rospy.loginfo('Recharging')
        time.sleep(self.recharging_time)
        return 'done'





def main():
    global OI_client, scanner_client, planner_client, controller_client
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm_main = StateMachine([])
    with sm_main:
        #---------- MAPPING STATE ----------
        StateMachine.add('MAPPING', Mapping(),
                               transitions={'map_loaded':'MONITORING'})

        #---------- MONITORING STATE ----------
        sm_monitoring = StateMachine(outcomes=['monitoring_done','battery_low'])
        with sm_monitoring:
                StateMachine.add('MOVE_TO_MONITOR', Move(type = 'monitor'),
                                 transitions={'done':'MONITOR',
                                              'preempted':'battery_low'})
                StateMachine.add('MONITOR', Monitor(),
                                 transitions={'done':'monitoring_done',})
                                              #'preempted':'preempted'})
                                              
        StateMachine.add('MONITORING', sm_monitoring,
                               transitions={'monitoring_done':'MONITORING',
                                            'battery_low':'RECHARGING'})
        #---------- RECHARGE STATE ----------
        sm_recharge = StateMachine(outcomes=['recharge_done'])
        with sm_recharge:
                StateMachine.add('MOVE_TO_RECHARGE', Move(type = 'recharge'),
                                 transitions={'done':'RECHARGE',
                                              'preempted':'recharge_done'})
                StateMachine.add('RECHARGE', Recharge(),
                                 transitions={'done':'recharge_done',})
                                              #'preempted':'preempted'})

        StateMachine.add('RECHARGING', sm_recharge,
                               transitions={'recharge_done':'MONITORING'})


    # Create and start the introspection server for visualization
    sis = IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()


    OI_client = actionlib.SimpleActionClient('OntologyInterface',
                                             assignment_1.msg.OICommandAction)

    scanner_client = actionlib.SimpleActionClient('action_scanner',
                                             assignment_1.msg.SMCommandAction)

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
