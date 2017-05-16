#!/usr/bin/env python
import fetch_api
import rospy
import pickle
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from barbot.msg import PoseNames
from barbot.msg import UserAction
import geometry_msgs.msg
from barbot.srv import *

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ActionServiceActuator(object):
    """
    ActionServiceActuator waits for service, gets the desired value, carries out the required action
    """
    def __init__(self):
        self._grip = fetch_api.Gripper()
        self._arm = fetch_api.Arm()
        pass


def main():
    rospy.init_node('bar_bot_actuator')
    


if __name__ == '__main__':
    main()
