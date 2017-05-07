#! /usr/bin/env python

import rospy
import pickle
import numpy as np
import actionlib
import tf
import fetch_api
from geometry_msgs.msg import PoseStamped
from pose_executable import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def transform_to_pose(matrix):
    pose = Pose()
    quat_from_mat = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
    vector = np.dot(matrix, np.array([0, 0, 0, 1]))
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
    return base_link_mat

def usage():
    print 'Usage: rosrun applications load_file_actions.py <filename.p>'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def getPoseMoveTo(pose_action, markers):
    for marker in markers:
        if pose_action.frame == marker.id:
            wrist2 = makeMatrix(pose_action.pose) 
            tag2 = makeMatrix(pose_action.arPose)  
            tag2 = tf.transformations.inverse_matrix(tag)
            result = np.dot(wrist2, tag2)
            result = np.dot(marker.pose.pose, result)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = transform_to_pose(result)
    return pose_stamped


def main():
    rospy.init_node("load_file_actions")
    wait_for_time()
    pose_execs = None

    argv = rospy.myargv()
    if len(argv) < 2:
        usage()
        return
    fileName = argv[1]
    try:
        pose_execs = pickle.load(open(fileName, "rb"))
    except:
        print 'Invalid File'
        usage()
        return
    print 'file loaded'
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    controller_client.send_goal(goal)
    controller_client.wait_for_result()
    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()
    for pose_actions in pose_execs:
        if pose_actions.action == PoseExecutable.OPEN:
            gripper.open()
        elif pose_actions.action == PoseExecutable.CLOSE:
            gripper.close()
        elif pose_actions.action == PoseExecutable.MOVETO:
            if pose_actions.frame == 'base_link':
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = pose_actions.pose
            else:
                pose_stamped = getPoseMoveTo(pose_actions, reader.markers)
            error = arm.move_to_pose(pose_stamped, allowed_planning_time=20)
            if error is not None:
                print 'error moving to {}, please try again, exiting'.format(pose_actions.pose)
                return
        else:
            print 'invalid command {}'.format(pose_actions.action)
