#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import geometry_msgs.msg
from pose_executable import *
import sys
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState, QueryControllerStatesAction
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import tf
import actionlib
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

class PoseSaver(object):
    def __init__(self):
        self.actions = []
        self.reader = ArTagReader()
        rospy.sleep(0.1)
        sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.reader.callback)
        rospy.sleep(0.1)
        self.tf_listener = tf.TransformListener()
        self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        self.gripper =  fetch_api.Gripper()

    def create_program(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def save_pose(self):
        goal = Pose()
        
        (transform, rotation) = self.tf_listener.lookupTransform('/base_link', '/wrist_roll_link', rospy.Time(0))
        goal.position.x = transform[0]
        goal.position.y = transform[1]
        goal.position.z = transform[2]
        goal.orientation.x = rotation[0]
        goal.orientation.y = rotation[1]
        goal.orientation.z = rotation[2]
        goal.orientation.w = rotation[3]
        print 'Is the pose relative to base frame or to a tag?'
        print 'base_frame'
        count = 0
        for marker in self.reader.markers:
            print "Index %(first)d is: tag %(second)d" % {"first": count, "second":marker.id}
            count += 1

        lines = raw_input('')
        if lines == 'base_frame':
            action = PoseExecutable(PoseExecutable.MOVETO, 'base_link', goal)
        else:
            tag_id = int(lines)
            marker = self.reader.markers[tag_id]
            action = PoseExecutable(PoseExecutable.MOVETO, marker.id, goal)
            #goal.header.frame
        
        self.actions.append(action)

        

    def open_gripper(self):
        action = PoseExecutable(PoseExecutable.OPEN, None, None)
        self.actions.append(action)
        self.gripper.open()



    def close_gripper(self):
        action = PoseExecutable(PoseExecutable.CLOSE, None, None)
        self.actions.append(action)
        self.gripper.close()

    def save_program(self, filename):
        pickle.dump(self.actions, open(filename, "wb" ))


def print_usage():
    print 'Commands:'
    print ' create_program: relaxes robot'
    print ' open_gripper: opens the gripper'
    print ' close_gripper: closes the gripper at max force'
    print ' save_pose: saves the current pose relative to'
    print ' save_program <name>: saves the current program to <name>'
    print ' help: Show this list of commands'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('pbD')
    # try:
    #     map_list_data = pickle.load(open(PICKLE_FILE, "rb"))
    # except:
    #     print 'Pickle File Empty'
    #     map_list_data = {}
    #     pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )
    
    wait_for_time()
    pose_saver = PoseSaver()

    argv = rospy.myargv()
    while True:
        lines = sys.stdin.readline()
        command = lines.strip()

        if command == 'create_program':
            pose_saver.create_program()
        elif command == 'save_pose':
            pose_saver.save_pose()
        elif command == 'open_gripper':
            pose_saver.open_gripper()
        elif command == 'close_gripper':
            pose_saver.close_gripper()
        elif command.startswith('save_program'):
            commandList = lines.split(' ', 1)
            name = commandList[1].strip()
            pose_saver.save_program(name)
        elif command == 'help':
            print_usage()
        else:
            print_usage()

if __name__ == '__main__':
    print 'Welcome to the programming by demonstration system!'   
    print_usage()         
    main()