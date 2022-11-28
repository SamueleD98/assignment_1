#!/usr/bin/env python

"""
.. module::robot_state
    :platform: Unix
    :synopsis: Python module for the state machine

ROS node which keeps track of the robot pose and battery status.

Being a simpler version, for the documentation please refer to `the original code <https://github.com/buoncubi/arch_skeleton>`_


"""

import rospy
import random
import threading
import time
from std_msgs.msg import String, Bool
from assignment_1.msg import Point
from assignment_1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse


class RobotState:
    def __init__(self):
        # Initialise this node
        rospy.init_node('robot_state')

        # Initialise robot position.
        self._pose = Point(x=0.0, y=0.0)

        # Initialise battery level
        self._battery_low = False

        print('robot_state')

        # Initialise randomness
        self._random_battery_time = rospy.get_param("/battery_time", [15.0, 40.0])

        # Define services
        rospy.Service("/get_pose", GetPose, self.get_pose)
        rospy.Service("/set_pose", SetPose, self.set_pose)

        # Start publisher on a separate thread
        th = threading.Thread(target=self._battery_status)
        th.start()

    def set_pose(self, request):
        if request.pose is not None:
            self._pose = request.pose
        else:
            rospy.logerr('Cannot set an unspecified robot position')
        return SetPoseResponse()

    def get_pose(self, request):
        response = GetPoseResponse()
        response.pose = self._pose
        return response


    def _battery_status(self):
        # Define a publisher
        pub = rospy.Publisher('battery_status', Bool, queue_size=1)

        while not rospy.is_shutdown():
            # Publish battery level
            pub.publish(Bool(self._battery_low))
            # Wait for simulate battery usage.
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low


if __name__ == "__main__":
    # Instantiate the node manager class and wait
    RobotState()
    rospy.spin()
