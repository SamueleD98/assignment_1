#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from assignment_1.srv import *
from assignment_1.msg import PlanAction
from assignment_1.msg import PlanGoal
from assignment_1.msg import PlanResult
import assignment_1
from std_msgs.msg import String
from std_msgs.msg import Bool

last_battery_low_update = False

class Mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['map_loaded'],
                             )

    def execute(self, userdata):
        print(' ')
        rospy.loginfo('Executing state MAPPING')

        goal = assignment_1.msg.SMCommandGoal()
        goal.command = 'load_map'
        Scanner_client.send_goal(goal)
        Scanner_client.wait_for_result()

        #print(Scanner_client.get_result().res)
        time.sleep(10)
        return 'map_loaded'
        #return 'map_loaded'


class Monitoring(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done','battery_low'],
                             )


    def execute(self, userdata):
        print(' ')
        rospy.loginfo('Executing state MONITORING ')

        get_plan = assignment_1.msg.SMCommandGoal()
        get_plan.command = 'monitoring'
        Planner_client.send_goal(get_plan)
        #while(Planner_client.get_state() == 0): {}
        while(Planner_client.get_state() != 3):
            if self.preempt_requested():
                Planner_client.cancel_all_goals()
                self.service_preempt()
                return 'battery_low'
        if self.preempt_requested():
            print(Planner_client.get_state())
            Planner_client.cancel_all_goals()
            self.service_preempt()
            return 'battery_low'
        #print('waiting')
        Planner_client.wait_for_result()

        target = Planner_client.get_result().res

        rospy.loginfo('Going to '+target)

        control_monitoring = assignment_1.msg.SMCommandGoal()
        control_monitoring.command = target
        Controller_client.send_goal(control_monitoring)
        #while(Controller_client.get_state() == 0): {}
        while(Controller_client.get_state() != 3):
            if self.preempt_requested():
                Controller_client.cancel_all_goals()
                self.service_preempt()
                return 'battery_low'
            #print('active')
        Controller_client.wait_for_result()
        result = Controller_client.get_result().res

        return result


class Recharge(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             )


    def execute(self, userdata):
        global last_battery_low_update
        print(' ')
        rospy.loginfo('Executing state RECHARGE ')

        get_plan = assignment_1.msg.SMCommandGoal()
        get_plan.command = 'recharge'
        Planner_client.send_goal(get_plan)
        while(Planner_client.get_state() != 3):
            if self.preempt_requested():
                Planner_client.cancel_all_goals()
                self.service_preempt()
                return 'battery_low'
        if self.preempt_requested():
            Planner_client.cancel_all_goals()
            self.service_preempt()
            return 'battery_low'
        Planner_client.wait_for_result()
        target = Planner_client.get_result().res

        rospy.loginfo('Going to '+target)

        control_monitoring = assignment_1.msg.SMCommandGoal()
        control_monitoring.command = target
        Controller_client.send_goal(control_monitoring)
        while(Controller_client.get_state() != 3):
            if self.preempt_requested():
                Controller_client.cancel_all_goals()
                self.service_preempt()
                return 'battery_low'
        if self.preempt_requested():
            Controller_client.cancel_all_goals()
            self.service_preempt()
            return 'battery_low'
        Controller_client.wait_for_result()
        result = Controller_client.get_result().res

        rospy.loginfo('Recharging..')

        control_monitoring = assignment_1.msg.SMCommandGoal()
        control_monitoring.command = 'recharge'
        Controller_client.send_goal(control_monitoring)
        while(Controller_client.get_state() != 3):
            if self.preempt_requested():
                Controller_client.cancel_all_goals()
                self.service_preempt()
                return 'battery_low'
        if self.preempt_requested():
            Controller_client.cancel_all_goals()
            self.service_preempt()
            return 'battery_low'
        Controller_client.wait_for_result()
        result = Controller_client.get_result().res

        last_battery_low_update = False
        return result



# gets called when ANY child state terminates
def child_term_cb(outcome_map):
  return True

# gets called when ALL child states are terminated
def out_cb_monitoring(outcome_map):
   #print(outcome_map['CHECK'])
   if outcome_map['CHECK'] == 'invalid':
      return 'battery_low'
   else:
      return 'done'

def out_cb_recharge(outcome_map):
    return 'done'

def monitor_cb(ud, msg):
    global last_battery_low_update
    #string =
    rospy.loginfo('BATTERY STATUS UPDATE: is battery low? --> ' + str(msg.data))
    # rospy.loginfo('BATTERY STATUS UPDATE: is battery low? --> ')
    # rospy.loginfo(msg.data)
    if last_battery_low_update == msg.data :
        #rospy.loginfo('BATTERY STATUS UPDATE: is battery low? --> '+msg.data)
        return True
    #print('change status!!')
    last_battery_low_update = msg.data
    return False


def main():
    global Scanner_client, Planner_client, Controller_client
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['sm_outcome'])

    # Open the container
    with sm_top:
        #---------- MAPPING STATE ----------
        smach.StateMachine.add('MAPPING', Mapping(),
                               transitions={'map_loaded':'MONITORING'})

        #---------- MONITORING (CONCURRENCE) STATE ----------
        sm_con_monitoring = smach.Concurrence(outcomes=['done','battery_low'],
                                   default_outcome='done',
                                   child_termination_cb = child_term_cb,
                                   outcome_cb = out_cb_monitoring)
        with sm_con_monitoring:
            smach.Concurrence.add('MONITORING_EXE', Monitoring())
            smach.Concurrence.add('CHECK',
                                  smach_ros.MonitorState("/battery_status", Bool, monitor_cb))
        smach.StateMachine.add('MONITORING', sm_con_monitoring,
                               transitions={'done':'MONITORING',
                                            'battery_low':'RECHARGE'})

        #---------- RECHARGE (CONCURRENCE) STATE ----------
        sm_con_recharge = smach.Concurrence(outcomes=['done'],
                                   default_outcome='done',
                                   child_termination_cb = child_term_cb,
                                   outcome_cb = out_cb_recharge)
        with sm_con_recharge:
            smach.Concurrence.add('RECHARGE_EXE', Recharge())
            smach.Concurrence.add('CHECK',
                                  smach_ros.MonitorState("/battery_status", Bool, monitor_cb))
        smach.StateMachine.add('RECHARGE', sm_con_recharge,
                               transitions={'done':'MONITORING'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()


    Scanner_client = actionlib.SimpleActionClient('action_scanner',
                                             assignment_1.msg.SMCommandAction)

    Planner_client = actionlib.SimpleActionClient('action_planner',
                                             assignment_1.msg.SMCommandAction)

    Controller_client = actionlib.SimpleActionClient('action_controller',
                                             assignment_1.msg.SMCommandAction)

    Scanner_client.wait_for_server()
    Planner_client.wait_for_server()
    Controller_client.wait_for_server()


    # Execute the state machine
    sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
