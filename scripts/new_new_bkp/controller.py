#! /usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
import random
from assignment_1.srv import *
from assignment_1.msg import ControlResult
from armor_api.armor_client import ArmorClient

class ControllerAction(object):
    def __init__(self):
        self.motion_time = rospy.get_param("motion_time", [0.1, 0.2])

        self._as = SimpleActionServer('action_controller',
                                      assignment_1.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)

        self._as.start()
        print('controller ok')

    def execute_callback(self, goal):

        if goal is None or goal.via_points is None:
            rospy.logerr('No via points provided! This service will be aborted!')
            self._as.set_aborted()
            return
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo('Service has been cancelled by the client!')
                self._as.set_preempted()
                return
            delay = random.uniform(self.motion_time[0], self.motion_time[1])
            rospy.sleep(delay)
            self._set_pose_client(point)

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = point
        rospy.loginfo('Motion control successes.')
        self._as.set_succeeded(result)
        return  # Succeeded.

    def _set_pose_client(self, pose):
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service('/set_pose')
        try:
            rospy.loginfo('Set new robot position')
            service = rospy.ServiceProxy('/set_pose', SetPose)
            service(pose)  # The `response` is not used.
        except rospy.ServiceException as e:
            rospy.logerr('Server cannot set current robot position')



if __name__ == '__main__':
    rospy.init_node('controller')
    server = ControllerAction()
    rospy.spin()
