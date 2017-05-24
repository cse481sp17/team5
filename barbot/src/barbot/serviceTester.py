#!/usr/bin/env python

from barbot.srv import *
import rospy
import signal
import sys

def move_to_perception_client(item):
    print 'getting {}'.format(item)
    rospy.wait_for_service('move_to_perception')
    try:
        move_to_perception = rospy.ServiceProxy('move_to_perception', MoveToPerception)
        resp = move_to_perception(item)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def print_result(resp1):
    rospy.loginfo('x = %f, y = %f, z = %f, item = %s', resp1.x, resp1.y, resp1.z, resp1.item)
    
def print_usage():
    print 'Commands:'
    print '  get_cup: gets x y z of cup.'
    print '  get_table: gets x y z of table'
    print '  help: Show this list of commands'

def signal_handler(signal, frame):
    print('Exiting Safely!')
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('move_to_tester')
    print_usage()

    signal.signal(signal.SIGINT, signal_handler)
    while True:
        lines = sys.stdin.readline()
        command = lines.strip()
        if command == 'get_cup':
            resp1 = move_to_perception_client(MoveToPerceptionRequest.CUP)
            print_result(resp1)
        elif command == 'get_table':
            resp1 = move_to_perception_client(MoveToPerceptionRequest.TABLE)
            print_result(resp1)
        elif command == 'help':
            print_usage()
        else:
            print_usage()