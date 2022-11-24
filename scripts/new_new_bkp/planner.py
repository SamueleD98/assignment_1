#! /usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionServer
import time
import roslib
import assignment_1
import assignment_1.msg
import random
from assignment_1.srv import GetPose
from assignment_1.msg import OICommandResult, Point, PlanResult
from armor_api.armor_client import ArmorClient

class PlannerAction(object):
    def __init__(self):
        self.planning_time = rospy.get_param("planning_time", [0.1, 0.2])

        self._as = SimpleActionServer('action_planner',
                                      assignment_1.msg.PlanAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)

        self._as.start()
        print('planner ok')

    def execute_callback(self, goal):
        start_point = self._get_pose_client()
        target_point = goal.target
        if start_point is None or target_point is None:
            rospy.logerr('Cannot have `None` start point nor target_point. This service will be aborted!')
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        result = PlanResult()
        result.via_points.append(start_point)
        delay = random.uniform(self.planning_time[0], self.planning_time[1])
        rospy.sleep(delay)

        number_of_points = random.randint(1, 10)

        for i in range(1, number_of_points):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo('Server has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()
                return
            # Generate a new random point of the plan.
            new_point = Point()
            new_point.x = random.uniform(0, 100)
            new_point.y = random.uniform(0, 100)
            result.via_points.append(new_point)
            if i < number_of_points - 1:
                # Wait to simulate computation.
                delay = random.uniform(self.planning_time[0], self.planning_time[1])
                rospy.sleep(delay)
            else:
                # Append the target point to the plan as the last point.
                result.via_points.append(target_point)
        rospy.loginfo('Motion plan succeeded!')
        self._as.set_succeeded(result)

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
