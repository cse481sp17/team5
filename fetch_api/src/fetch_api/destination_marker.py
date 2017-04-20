#!/usr/bin/env python


from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

class DestinationMarker(object):
    def __init__(self, server, x, y, name, driver):
        # ... Initialization, marker creation, etc. ...
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.orientation.w = 1

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(box_marker)
        int_marker.controls.append(button_control)
        self._server = server
        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()

    def _callback(self, msg):
         # TODO: How do you get the interactive marker given msg.marker_name?
         # See the InteractiveMarkerServer documentation
         interactive_marker = ???
         position = interactive_marker.pose.position
         rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, position.x, position.y, position.z))
         self._driver.goal = new_position # Updates the Driver's goal.