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

current_amcl = None

def print_usage():
    print 'Commands:'
    print '  home: Sends me to my home.'
    print '  beer: fills a glass of beer'
    print '  deliver <location>: fills and delivers a glass of beer'
    print '  help: Show this list of commands'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def amcl_callback(msg):
    global current_amcl
    current_amcl = msg


def main():
    rospy.init_node('bar_controller')
    amcl_sub = rospy.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, amcl_callback)


if __name__ == '__main__':
    print 'Hi I am Michael, tell me what you want me to do:'
    print_usage()
    main()
