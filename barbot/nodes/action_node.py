#!/usr/bin/env python

import fetch_api
import rospy
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
        pass


def main():
    rospy.init_node('bar_bot_actuator')
    pass


if __name__ == '__main__':
    main()
