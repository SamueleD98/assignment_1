#!/usr/bin/env python

import rospy
import random
import threading
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from assignment_1.srv import GetLocation, GetLocationResponse, SetLocation, SetLocationResponse

class RobotState:
    def __init__(self):
        # Initialise this node
        rospy.init_node('robot_state')

        # Initialise robot position
        #self._location = None

        # Initialise battery level
        self._battery_low = False

        # Initialise randomness
        try:
            self._random_battery_time = rospy.get_param("/battery_time")
        except:
            print('WARNING: No params on server, using default ones')
            self._random_battery_time = [15.0, 40.0]


        # Define services
        #rospy.Service("/get_location", GetLocation, self.get_location)
        #rospy.Service("/set_location", SetLocation, self.set_location)

        # Start publisher on a separate thread
        th = threading.Thread(target=self._battery_status)
        th.start()


    def set_location(self, request):
        if request.location is not None:
            self._location = request.location
        return SetLocationResponse()


    def get_location(self, request):
        if self._location is not None:
            response = GetLocationResponse()
            response.location = self._location
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

#---------------------------

# def battery_status():
#     msg = String()
#     msg.data = random.choice(['battery_low','battery_high'])
#     return msg
#
# def robot_state():
#     pub = rospy.Publisher('battery_status', String, queue_size=1)
#     rospy.init_node('robot_state')
#     #rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         pub.publish(battery_status())
#         time.sleep(120000)
#
#
#
# if __name__ == '__main__':
#     try:
#         robot_state()
#     except rospy.ROSInterruptException:
#         pass
