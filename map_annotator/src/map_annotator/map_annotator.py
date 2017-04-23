#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import geometry_msgs.msg
import sys

PICKLE_FILE='map_annotated.p'
map_list_data = {}
current_amcl = None

def print_usage():
    print 'Commands:'
    print '  list: List saved poses.'
    print '  save <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.'
    print '  delete <name>: Delete the pose given by <name>.'
    print '  goto <name>: Sends the robot to the pose given by <name>.'
    print '  help: Show this list of commands'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_list():
    print 'Poses:'

    for key in map_list_data.keys():
        print ' ' + key

def save_new_pose(name):
    map_list_data[name] = current_amcl
    pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )

def amcl_callback(msg):
    global current_amcl
    current_amcl = msg

def delete_pose(name):
    if name in map_list_data:
        map_list_data.pop(name)
        pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )        
    else:
        print 'No such pose \'{}\''.format(name)

def goto_pose(name):
    if name in map_list_data:
        target_pose = map_list_data[name]
        message = geometry_msgs.msg.PoseStamped()
        message.header = target_pose.header
        message.pose = target_pose.pose.pose
        target_pose_pub.publish(message)
    else:
        print 'No such pose \'{}\''.format(name)

def main():
    global map_list_data
    rospy.init_node('map_annotator')
    try:
        map_list_data = pickle.load(open(PICKLE_FILE, "rb"))
    except:
        print 'Pickle File Empty'
        map_list_data = {}
        pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )
    
    wait_for_time()
    global target_pose_pub
    target_pose_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)

    amcl_sub = rospy.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, amcl_callback)
    argv = rospy.myargv()
    while True:
        lines = sys.stdin.readline()
        command = lines.strip()
        if command != 'list' and command != 'help':
            commandList = lines.split(' ', 1)
            name = commandList[1].strip()
            command = commandList[0].strip()

        if command == 'list':
            print_list()
        elif command == 'save':
            save_new_pose(name)
        elif command == 'delete':
            delete_pose(name)
        elif command == 'goto':
            goto_pose(name)
        elif command == 'help':
            print_usage()
        else:
            print_usage()

if __name__ == '__main__':
    print 'Welcome to the map annotator!'   
    print_usage()         
    main()