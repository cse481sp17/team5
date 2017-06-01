#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from barbot.srv import *
import numpy as np
import actionlib
import tf
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from pose_executable import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState
import tf.transformations as tft

# PICKLE_FILE_PUT_DISPENSER='put_dispenser.p'
# PICKLE_FILE_DISPENSE='dispenser.p'
# PICKLE_FILE_GET_GLASS='get_glass.p'


# pickle_files = [PICKLE_FILE_PUT_DISPENSER, PICKLE_FILE_DISPENSE, PICKLE_FILE_GET_GLASS]
PICKLE_FILE = 'ar_tags.p'


OFFSET_X = 0.10
OFFSET_Z = 0.10 # should be the hight of the cup
GRIPPER_OFFSET = 0.171
DISPENSE_TIME = 3.0
pre_pose_list = [-1.605528329547318, 1.41720603380179, 2.018610841968549, 1.5522558117738399, -1.5635699410855368, 0.7653977094751401, -1.3914909133500242]
move_arm_to_the_right = [-1.605528329547318, 1.41720603380179, 2.018610841968549, 1.5522558117738399, -1.5635699410855368, 0.7653977094751401, -1.3914909133500242]
# test [-1.3705930364835375, 1.280469534625107, 3.138322253626793, 1.102770685640171, -1.2980107896440538, 0.5342823853513483, -1.8845609229070917]


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

class ArmServer(object):
    def __init__(self):
        self._grip = fetch_api.Gripper()
        self._base = fetch_api.Base()
        self._arm = fetch_api.Arm()
        self._torso = fetch_api.Torso()
        self._torso.set_height(0.4)
        self._head = fetch_api.Head()
        self._head.pan_tilt(0, 0.0)
        self.actions = None
        try:
            self.actions = pickle.load(open(PICKLE_FILE, "rb"))
            print '{} loaded.'.format(PICKLE_FILE)
        except:
            print '{} could not be loaded.'.format(PICKLE_FILE)

        self._reader = ArTagReader()
        self._sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._reader.callback)

    def set_prepose(self):
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(pre_pose_list))

    def set_arm_to_the_right(self):
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(move_arm_to_the_right))

    def lookup(self):
        self._head.pan_tilt(0, 0)
    
    def lookdown(self):
        self._head.pan_tilt(0, 0.65)

    def findGlass(self, request):
        current_pose = Pose(orientation=Quaternion(0,0,0,1))
        current_pose.position.x = request.x - GRIPPER_OFFSET
        current_pose.position.y = request.y
        current_pose.position.z = request.z + 0.03

        self._base.stop()
        self._head.pan_tilt(0, 0.65)

        if (request.item == 'cup'):
            self._head.pan_tilt(0, 0.65)
            self._grip.open()
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose = copy.deepcopy(current_pose)
            goal.pose.position.x -= OFFSET_X
            self._arm.move_to_pose(goal)
            goal.pose.position.x += OFFSET_X
            self._arm.move_to_pose(goal)
            self._grip.close(45)
            goal.pose.position.z += OFFSET_Z * 1.5
            self._arm.move_to_pose(goal)

            goal.pose.position.x -= OFFSET_X * 2
            self._arm.move_to_pose(goal)

            # goal.pose.position.y += OFFSET_X
            # self._arm.move_to_pose(goal)

            goal.pose.position.z -= OFFSET_Z
            self._arm.move_to_pose(goal)

            ## find the dispenser and set the glass correctly on it
            self.load_fiducial_marker_actions()

            # self.set_prepose()
            #self.set_arm_to_the_right()
        else:
            #self.set_prepose()
            self._head.pan_tilt(0, 0.65)
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose = copy.deepcopy(current_pose)
            goal.pose.position.x -= OFFSET_X
            goal.pose.position.z += 2 * OFFSET_Z
            self._arm.move_to_pose(goal)
            goal.pose.position.z -= (OFFSET_Z + 0.07)
            error = self._arm.move_to_pose(goal)

            if error is None:
                self._grip.open()
                goal.pose.position.x -= OFFSET_X * 2
                self._arm.move_to_pose(goal)
                return True
            else:
                return False

    def transform_to_pose(self, matrix):
        pose = Pose()
        quat_from_mat = tft.quaternion_from_matrix(matrix)
        pose.orientation = Quaternion(quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
        x = matrix[0][3]
        y = matrix[1][3]
        z = matrix[2][3]
        pose.position = Point(x, y, z)
        return pose

    def makeMatrix(self, pose):
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


    def load_fiducial_marker_actions(self, amount=3.0):
        pose_actions = copy.deepcopy(self.actions)

        pre_count = 0
        count = 0
        closed = False
        # Run through each of the actions
        for pose_action in pose_actions:
            count += 1
            if pose_action.actionType == PoseExecutable.OPEN:
                self._grip.open()
            elif pose_action.actionType == PoseExecutable.CLOSE:
                self._grip.close(50)
                if pre_count == 0:
                    closed = True
                    pre_count = count
            elif pose_action.actionType == PoseExecutable.MOVETO:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                if pose_action.relativeFrame == 'base_link':
                    pose_stamped.pose = pose_action.pose
                else:
                    for marker in self._reader.markers:
                        if pose_action.relativeFrame == marker.id:
                            wrist2 = self.makeMatrix(pose_action.pose) 
                            tag = self.makeMatrix(pose_action.arPose)  
                            tag2 = tf.transformations.inverse_matrix(tag)
                            result = np.dot(tag2, wrist2)
                            result2 = np.dot(self.makeMatrix(marker.pose.pose), result)

                            pose_stamped = PoseStamped()
                            pose_stamped.header.frame_id = "base_link"
                            pose_stamped.pose = self.transform_to_pose(result2)
                error = self._arm.move_to_pose(pose_stamped, allowed_planning_time=40, num_planning_attempts=20)
                if error is not None:
                    print 'Error moving to {}.'.format(pose_action.pose)
                    
                if closed and count - pre_count == 2:
                    rospy.sleep(amount)
                    closed = False
            else:
                print 'invalid command {}'.format(pose_action.action)
