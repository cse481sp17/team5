#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MenuEntry
import copy

GRIPPER_OFFSET = 0.166
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2
GRIPPER_MOVETO = 3
GRIPPER_AUTOPICK = 4
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


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



def createMarker():
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
    gripper_center.pose.position.x += GRIPPER_OFFSET
    gripper_left.pose.position.x += GRIPPER_OFFSET
    gripper_right.pose.position.x += GRIPPER_OFFSET

    gripper_left.pose.position.y = gripper_center.pose.position.y - 0.05
    gripper_right.pose.position.y = gripper_center.pose.position.y+ 0.05
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

    gripper_markers = createMarker()
    int_control.interaction_mode = InteractiveMarkerControl.MENU
    int_control.markers.extend(gripper_markers)
    int_marker.scale = 0.25
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
    move_gripper.title = 'MOVE TO'
    menu_entries.append(move_gripper)
    int_marker.menu_entries.extend(menu_entries)
    return int_marker