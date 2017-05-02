#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MenuEntry
import numpy as np
import copy
import tf.transformations as tft


GRIPPER_OFFSET = 0.166
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2
GRIPPER_MOVETO = 3
GRIPPER_AUTOPICK = 4
OFFSET_X = -0.15
OFFSET_Z = 0.25
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


def transform_to_pose(matrix):
    pose = Pose()
    quat_from_mat = tft.quaternion_from_matrix(matrix)
    # print quat_from_mat
    pose.orientation = Quaternion(quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
    vector = np.dot(matrix, np.array([0, 0, 0, 1]))
    x = matrix[0][3]
    y = matrix[1][3]
    z = matrix[2][3]
    # print vector
    # pose.position = Point(vector[0], vector[1], vector[2])
    pose.position = Point(x, y, z)
    return pose

def makeMatrix(pose):
    orien = pose.orientation
    base_link_mat = tft.quaternion_matrix([orien.x, orien.y, orien.z, orien.w])
    base_link_mat[0][3] = pose.position.x
    base_link_mat[1][3] = pose.position.y
    base_link_mat[2][3] = pose.position.z
    return base_link_mat

def b_in_marker(frame_a_marker, frame_b):
    frame_a_marker_pose = copy.deepcopy(frame_a_marker.pose)
    #frame_a_marker_pose.position.x += GRIPPER_OFFSET
    frame_b_mat = makeMatrix(frame_b)
    base_link_mat = makeMatrix(frame_a_marker_pose)
    dot = np.dot(frame_b_mat, base_link_mat)
    return transform_to_pose(dot)


def make_6dof_controls():
    dof_controls = []
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    dof_controls.append(copy.deepcopy(control))
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    dof_controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    dof_controls.append(copy.deepcopy(control))
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    dof_controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    dof_controls.append(copy.deepcopy(control))
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    dof_controls.append(control)
    return dof_controls



def createMarker(x_pos, z_pos):
    # gen marker
    gripper_center = Marker()
    gripper_left = Marker()
    gripper_right = Marker()
    #mesh resource
    gripper_center.type = Marker.MESH_RESOURCE
    gripper_left.type = Marker.MESH_RESOURCE
    gripper_right.type = Marker.MESH_RESOURCE
    # add resource
    gripper_center.mesh_resource = GRIPPER_MESH
    gripper_left.mesh_resource = L_FINGER_MESH
    gripper_right.mesh_resource = R_FINGER_MESH
    # add color to gripper
    gripper_center.color.r = 1.0
    gripper_left.color.r = 1.0
    gripper_right.color.r = 1.0
    gripper_center.color.a = 1.0
    gripper_left.color.a = 1.0
    gripper_right.color.a = 1.0
    gripper_center.scale.x = 1.0
    gripper_center.scale.y = 1.0
    gripper_center.scale.z = 1.0
    gripper_center.pose.position.x += GRIPPER_OFFSET + x_pos
    gripper_left.pose.position.x += GRIPPER_OFFSET + x_pos
    gripper_right.pose.position.x += GRIPPER_OFFSET + x_pos
    gripper_center.pose.position.z += z_pos
    gripper_left.pose.position.z += z_pos
    gripper_right.pose.position.z += z_pos
    
    gripper_left.pose.position.y = gripper_center.pose.position.y - 0.05
    gripper_right.pose.position.y = gripper_center.pose.position.y + 0.05
    return [gripper_center, gripper_left, gripper_right]


def getMarkersFromPose(poseStamped, preGraspStatus):
    int_marker = InteractiveMarker()
    int_marker.name = 'gripper'
    int_marker.header.frame_id = poseStamped.header.frame_id
    int_marker.pose = poseStamped.pose
    int_control = InteractiveMarkerControl()
    int_control.always_visible = True
    int_control.orientation.w = 1
    int_control.orientation.x = 0
    int_control.orientation.y = 1
    int_control.orientation.z = 0

    # ADD GRIPPER CONTROLS

    gripper_markers = createMarker(0, 0)
    int_control.interaction_mode = InteractiveMarkerControl.MENU
    int_control.markers.extend(gripper_markers)
    int_marker.scale = 0.35

    # ADD AUTO PICKING
    if preGraspStatus:
        #pre-grasp
        int_control.markers.extend(createMarker(OFFSET_X, 0))
        #lift-grasp
        int_control.markers.extend(createMarker(0, OFFSET_Z))
        #sample-box
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.position.x += GRIPPER_OFFSET        
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05
        box_marker.color.r = 1
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.5
        int_control.markers.append(box_marker)

    int_marker.controls.append(int_control)


    # ADD DOF
    dof_controls = make_6dof_controls()
    int_marker.controls.extend(dof_controls)

    # ADD MENU ITEMS
    menu_entries = []
    open_gripper = MenuEntry()
    open_gripper.id = GRIPPER_OPEN
    open_gripper.command_type = MenuEntry.FEEDBACK
    open_gripper.title = 'OPEN GRIPPER'
    menu_entries.append(open_gripper)
    close_gripper = MenuEntry()
    close_gripper.id = GRIPPER_CLOSE
    close_gripper.command_type = MenuEntry.FEEDBACK
    close_gripper.title = 'CLOSE GRIPPER'
    menu_entries.append(close_gripper)
    move_gripper = MenuEntry()
    move_gripper.id = GRIPPER_MOVETO
    move_gripper.command_type = MenuEntry.FEEDBACK
    if preGraspStatus:
        move_gripper.title = 'Commence AutoPick'        
    else:
        move_gripper.title = 'MOVE TO THIS POSE'
    menu_entries.append(move_gripper)
    int_marker.menu_entries.extend(menu_entries)

    return int_marker