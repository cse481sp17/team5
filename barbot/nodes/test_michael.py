#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import copy
import actionlib
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from barbot.srv import *
from botNavigation import NavigationServer
from botArm import ArmServer
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
def main():
    rospy.init_node('test_node')
    wait_for_time()

    # handle user actions
    rospy.wait_for_service('barbot/user_actions')
    test_action = rospy.ServiceProxy('barbot/user_actions', UserAction)
    test_action('make_drink', '1')

if __name__ == '__main__':
    main()