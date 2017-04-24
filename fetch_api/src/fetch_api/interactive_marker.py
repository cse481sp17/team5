#!/usr/bin/env python

import rospy
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

class PoseMarker(object):
    def __init__(self, server, name):
        # ... Initialization, marker creation, etc. ...
        self._driver = driver        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose.position.x = 0
        int_marker.pose.position.y = 0
        int_marker.pose.position.z = 0
        int_marker.pose.orientation.w = 1
        int_marker.scale = 1


        print name
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = 0.7
        arrow_marker.scale.y = 0.07
        arrow_marker.scale.z = 0.07
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append(arrow_marker)
        int_marker.controls.append(control)

        #int_marker_control.markers.append(arrow_marker)


        self._server = server
        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()

    def _callback(self, msg):
         # Get the interactive marker given msg.marker_name?
         # See the InteractiveMarkerServer documentation
         #if (msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        interactive_marker = self._server.get(msg.marker_name)
        position = interactive_marker.pose.position
        rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, position.x, position.y, position.z))