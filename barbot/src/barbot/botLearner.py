#!/usr/bin/env python
# TEAM 5
# CSE 481C
# Carl Ross, Mahir Kothary, Xukai Liu, Becky Leslie, Ling Jiaowang

import fetch_api
import rospy
import pickle
import geometry_msgs.msg
import sys
from command_bot import *

def print_usage():
    print 'Commands:'
    print '  save_home: save home location.'
    print '  save_beer: saves location for beer'
    print '  start_beer: starts recording beer filling'
    print '  save_delivery: saves the delivery location'
    print '  save_control: saves the bar controller'
    print '  help: Show this list of commands'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('bar_learner')


if __name__ == '__main__':
    print 'Hi there! Please train me and then I can be controlled:'
    print_usage()
    main()
