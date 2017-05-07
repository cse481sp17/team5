#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import geometry_msgs.msg
import sys

PICKLE_FILE='pbd_poses.p'
map_list_data = {}

class PoseSaver(object):
    def createProgram

def print_usage():
    print 'Commands:'
    print ' create_program: relaxes robot'
    print ' open_gripper: opens the gripper'
    print ' close_gripper: closes the gripper at max force'
    print ' save_pose <relation>: saves the current pose relative to <relation>'
    print ' save_program: saves the current program'
    print ' help: Show this list of commands'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



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
    # try:
    #     map_list_data = pickle.load(open(PICKLE_FILE, "rb"))
    # except:
    #     print 'Pickle File Empty'
    #     map_list_data = {}
    #     pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )
    
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

        if command == 'create_program':
            createProgram()
        elif command == 'save_pose':
            save_new_pose(name)
        elif command == 'open_gripper':
            delete_pose(name)
        elif command == 'close_gripper':
            goto_pose(name)
        elif command == 'save_program':

        elif command == 'help':
            print_usage()
        else:
            print_usage()

if __name__ == '__main__':
    print 'Welcome to the programming by demonstration system!'   
    print_usage()         
    main()