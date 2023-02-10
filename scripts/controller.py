#! /usr/bin/env python

"""
.. module::controller
    :platform: Unix
    :synopsis: Python module for the state machine

ROS node for implementing a controller.

Being a simpler version, for the documentation please refer to `the original code <https://github.com/buoncubi/arch_skeleton>`_

It calls the move_base package to make a plan and control the motion to a given target.

"""

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
import random
from assignment_1.srv import *
from assignment_1.msg import ControlResult, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class ControllerAction(object):
    """
    Class implementing the controller
    """
    def __init__(self):
        """
        Initialize the control server and the move_base client
        """

        self._as = SimpleActionServer('action_controller',
                                      assignment_1.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.move_base_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr('UNABLE TO CONNECT TO MOVE_BASE SERVER')

        self._as.start()
        rospy.loginfo('controller ok')

    def execute_callback(self, goal):
        """
        Called every time a goal comes:
        - Checks if there's a target into the goal std_msg
        - Send the goal to the move_base package
        - Waits while checking for preemption
        """
        print(' ')
        rospy.loginfo('Trying to reach the target')

        if goal is None or goal.target is None or goal.plan is None:
            rospy.logerr('No plan provided! This service will be aborted!')
            self._as.set_aborted()
            return

        #MOVE_BASE
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "map"
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.pose.position.x = goal.target.x
        mb_goal.target_pose.pose.position.y = goal.target.y
        mb_goal.target_pose.pose.orientation.w= 1.0;

        #move_base_client.send_goal(mb_goal)
        self.move_base_client.send_goal (mb_goal, feedback_cb=self.update_pose)
        rospy.loginfo('...')
        while(self.move_base_client.get_state() != 3):
            if self._as.is_preempt_requested():
                rospy.loginfo('Service has been cancelled by the client!')
                self._as.set_preempted()
                return

        self.move_base_client.wait_for_result()

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = goal.target
        rospy.loginfo('Motion control successes.')
        self._as.set_succeeded(result)
        return

    def update_pose(self, feedback):
        """
        Move_base feedback callback.
        Set the new pose in the robot_state node
        """
        pose = Point(x=feedback.base_position.pose.position.x,
                        y=feedback.base_position.pose.position.y)
        rospy.wait_for_service('/set_pose')
        try:
            #rospy.loginfo('Robot is now at ['+str(round(pose.x))+', '+str(round(pose.y))+']')
            service = rospy.ServiceProxy('/set_pose', SetPose)
            service(pose)  # The `response` is not used.
        except rospy.ServiceException as e:
            rospy.logerr('Server cannot set current robot position')

if __name__ == '__main__':
    rospy.init_node('controller')
    server = ControllerAction()
    rospy.spin()
