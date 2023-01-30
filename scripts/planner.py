#! /usr/bin/env python

"""
.. module::planner
    :platform: Unix
    :synopsis: Python module for the state machine

ROS node for implementing a planner.

Being this a simpler version, for the documentation please refer to the `original code <https://github.com/buoncubi/arch_skeleton>`_

NOTE: this is not used in latest version of the system (move_base used instead).

"""

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
import random
from assignment_1.srv import GetPose
from assignment_1.msg import Point, PlanResult, PlanAction
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

class PlannerAction(object):
    def __init__(self):
        self.planning_time = rospy.get_param("planning_time", [0.1, 0.2])

        self._as = SimpleActionServer('action_planner',
                                      PlanAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)

        self._as.start()
        rospy.loginfo('planner ok')

    def execute_callback(self, goal):
        print(' ')
        rospy.loginfo('Trying to make a plan')
        start_position = self._get_pose_client()
        target_position = goal.target

        # print(start_position)
        # print(target_position)

        if start_position is None or target_position is None:
            rospy.logerr('Cannot have `None` start point nor target point. This service will be aborted!')
            self._as.set_aborted()
            return

        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = rospy.Time.now()
        start_pose.pose.position.x = start_position.x
        start_pose.pose.position.y = start_position.y
        start_pose.pose.orientation.w= 1.0;

        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = target_position.x
        target_pose.pose.position.y = target_position.y
        target_pose.pose.orientation.w= 1.0;

        rospy.wait_for_service('/move_base/make_plan')
        try:
            rospy.loginfo('...')
            move_base_getPlan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            plan = move_base_getPlan(start=start_pose, goal=target_pose, tolerance=1.0)
            #return resp1.sum
        except rospy.ServiceException as e:
            rospy.logerr("Unable to build plan")
            self._as.set_aborted()

        rospy.loginfo('New plan!')
        self._as.set_succeeded(plan)

    #Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
    def _get_pose_client(self):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service('get_pose')
        try:
            # Call the service and get a response with the current robot position.
            service = rospy.ServiceProxy('get_pose', GetPose)
            response = service()
            pose = response.pose
            rospy.loginfo('Retrieving current robot position')
            return pose
        except rospy.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('planner')
    server = PlannerAction()
    rospy.spin()
