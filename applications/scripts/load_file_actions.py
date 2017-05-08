#!/usr/bin/env python

import rospy
import pickle
import numpy as np
import actionlib
import tf
import fetch_api
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from pose_executable import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState
import tf.transformations as tft

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def transform_to_pose(matrix):
    pose = Pose()
    quat_from_mat = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
    x = matrix[0][3]
    y = matrix[1][3]
    z = matrix[2][3]
    pose.position = Point(x, y, z)
    return pose

def makeMatrix(pose):
    orien = pose.orientation
    base_link_mat = tft.quaternion_matrix([orien.x, orien.y, orien.z, orien.w])
    base_link_mat[0][3] = pose.position.x
    base_link_mat[1][3] = pose.position.y
    base_link_mat[2][3] = pose.position.z
    base_link_mat[3][0] = 0
    base_link_mat[3][1] = 0
    base_link_mat[3][2] = 0
    base_link_mat[3][3] = 1
    return base_link_mat

def usage():
    print 'Usage: rosrun applications load_file_actions.py <filename.p>'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    pose_actions = None

    # Get the actions from the pickel file.
    argv = rospy.myargv()
    if len(argv) < 2:
        usage()
        return
    fileName = argv[1]
    try:
        pose_actions = pickle.load(open(fileName, "rb"))
        print '{} loaded.'.format(fileName)
    except:
        print '{} could not be loaded.'.format(fileName)
        usage()
        return
    
    rospy.init_node("load_file_actions")

    wait_for_time()
    print 'Time has begun.'

    # Start the arm.
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    rospy.sleep(1.0)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    controller_client.send_goal(goal)

    print 'Waiting for arm to start.'
    controller_client.wait_for_result()
    print 'Arm has been started.'


    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    print 'Arm and gripper instantiated.'

    rospy.sleep(1.0)

    # Run through each of the actions
    for pose_action in pose_actions:
        print 'Performing action.'
        if pose_action.actionType == PoseExecutable.OPEN:
            print 'Opening the gripper'
            gripper.open()
        elif pose_action.actionType == PoseExecutable.CLOSE:
            print 'Closing the gripper'
            gripper.close()
        elif pose_action.actionType == PoseExecutable.MOVETO:
            print 'Moving to location.'
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            if pose_action.relativeFrame == 'base_link':
                pose_stamped.pose = pose_action.pose
            else:
                for marker in reader.markers:
                    if pose_action.relativeFrame == marker.id:
                        wrist2 = makeMatrix(pose_action.pose) 
                        tag = makeMatrix(pose_action.arPose)  
                        tag2 = tf.transformations.inverse_matrix(tag)
                        result = np.dot(tag2, wrist2)
                        result2 = np.dot(makeMatrix(marker.pose.pose), result)

                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = "base_link"
                        pose_stamped.pose = transform_to_pose(result2)
            error = arm.move_to_pose(pose_stamped, allowed_planning_time=40, num_planning_attempts=20)
            if error is not None:
                print 'Error moving to {}.'.format(pose_action.pose)
        else:
            print 'invalid command {}'.format(pose_action.action)

if __name__ == '__main__':
    main()