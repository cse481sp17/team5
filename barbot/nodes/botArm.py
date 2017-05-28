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

PICKLE_FILE_PUT_DISPENSER='put_dispenser.p'
PICKLE_FILE_PUT_CLIENT='put_client.p'
PICKLE_FILE_DISPENSE='dispenser.p'
PICKLE_FILE_GET_GLASS='get_glass.p'
PICKLE_FILE_SET_PREPOSE='set_prepose.p'

OFFSET_X = 0.10
OFFSET_Z = 0.10 # should be the hight of the cup
GRIPPER_OFFSET = 0.171
DISPENSE_TIME = 3.0
pre_pose_list = [-1.605528329547318, 1.41720603380179, 2.018610841968549, 1.5522558117738399, -1.5635699410855368, 0.7653977094751401, -1.3914909133500242]


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
        self._head.pan_tilt(0, 0.65)
        self._reader = ArTagReader()
        self._sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._reader.callback)

    def set_prepose(self):
        #self.load_faducial_actions(PICKLE_FILE_SET_PREPOSE)
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(pre_pose_list))

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
            self._grip.close(50)
            goal.pose.position.z += OFFSET_Z
            self._arm.move_to_pose(goal)

            goal.pose.position.x -= OFFSET_X * 2
            self._arm.move_to_pose(goal)

            ## find the dispenser and set the glass correctly on it
            self.load_faducial_actions(PICKLE_FILE_PUT_DISPENSER)
            # dispense drink into the glass for a while
            self.load_faducial_actions(PICKLE_FILE_DISPENSE)
            # catch the glass again
            self.load_faducial_actions(PICKLE_FILE_GET_GLASS)

            # call the service and let the controller know the dispense work is done and now do the navigation
            # rospy.wait_for_service('barbot/action_done')
            # action_done = rospy.ServiceProxy('barbot/action_done', ActionDone)
            # action_done('goBack')
        else:
            goal = PoseStamped()
            goal.header.frame_id = 'base_link'
            goal.pose = copy.deepcopy(current_pose)
            goal.pose.position.z -= 0.03
            goal.pose.position.x -= OFFSET_X
            goal.pose.position.z += 2 * OFFSET_Z
            self._arm.move_to_pose(goal)
            goal.pose.position.z -= OFFSET_Z
            self._arm.move_to_pose(goal)
            self._grip.open()
            goal.pose.position.x -= OFFSET_X * 2
            self._arm.move_to_pose(goal)


            # rospy.wait_for_service('barbot/action_done')
            # action_done = rospy.ServiceProxy('barbot/action_done', ActionDone)
            # action_done('done')
            


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

    def load_faducial_actions(self, fileName):
        pose_actions = None
        try:
            pose_actions = pickle.load(open(fileName, "rb"))
            print '{} loaded.'.format(fileName)
        except:
            print '{} could not be loaded.'.format(fileName)
            usage()
            return

        count = 0

        # Run through each of the actions
        for pose_action in pose_actions:
            count += 1
            print 'Performing action.'
            if pose_action.actionType == PoseExecutable.OPEN:
                print 'Opening the gripper'
                self._grip.open()
            elif pose_action.actionType == PoseExecutable.CLOSE:
                print 'Closing the gripper'
                self._grip.close(50)
            elif pose_action.actionType == PoseExecutable.MOVETO:
                print 'Moving to location.'
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
                    
                if fileName is PICKLE_FILE_DISPENSE and count == 2:
                    rospy.sleep(DISPENSE_TIME)
            else:
                print 'invalid command {}'.format(pose_action.action)


# def main():

#     rospy.init_node('bot_arm')
#     wait_for_time()

#     # arm_server = ArmServer()
#     # # Arm_server.init
#     # reader = ArTagReader()
#     # sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
#     # # MOVE TO MICHAEL_NODE
#     # controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
#     # rospy.sleep(0.5)
#     # goal = QueryControllerStatesGoal()
#     # state = ControllerState()
#     # state.name = 'arm_controller/follow_joint_trajectory'
#     # state.state = ControllerState.RUNNING
#     # goal.updates.append(state)
#     # controller_client.send_goal(goal)

#     # print 'Waiting for arm to start.'
#     # controller_client.wait_for_result()

#     rospy.spin()

# if __name__ == '__main__':
#     main()