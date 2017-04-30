#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def createMarker():
    gripper_center = Marker()
    gripper_left = Marker()
    gripper_right = Marker()
    gripper_center.type = Marker.MESH_RESOURCE
    gripper_left.type = Marker.MESH_RESOURCE
    gripper_right.type = Marker.MESH_RESOURCE
    gripper_center.mesh_resource = GRIPPER_MESH
    gripper_left.mesh_resource = L_FINGER_MESH
    gripper_right.mesh_resource = R_FINGER_MESH
    return [gripper_center, gripper_left, gripper_right]


def getMarkersFromPose(poseStamped):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = poseStamped.header.frame_id
    int_marker.pose = poseStamped.pose
    int_control = InteractiveMarkerControl()
    int_control.markers = createMarker()
    int_marker.controls.append(int_control)
    return int_marker