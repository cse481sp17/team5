#!/usr/bin/env python

import rospy
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from navigation_node import NavigationServer
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction
import geometry_msgs.msg

pose_marker_server = None
nav_server = None
target_pose_pub = None

class PoseMarker(object):
    def __init__(self, server, name, pose_set):
        # ... Initialization, marker creation, etc. ...
        self.pose = pose_set
        if pose_set == None:
            self.pose = geometry_msgs.msg.Pose()
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose = self.pose


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
        arrow_marker.pose.position.z = 0.1

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
        interactive_marker = self._server.get(msg.marker_name)
        position = interactive_marker.pose

        if msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose = position

            nav_server.saveMarker(msg.marker_name, position)
            self._server.applyChanges()

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def handle_user_actions(message):
        name = message.name
        if message.command == 'create':
            new_pose_marker = PoseMarker(pose_marker_server, name, None)
            nav_server.saveMarker(name, new_pose_marker.pose)

        elif message.command == 'goto':

            if name in nav_server.pose_names_list:
                nav_server.goToMarker(name)

            else:
                print 'No such pose \'{}\''.format(name)

        elif message.command == 'delete':
            if name in nav_server.pose_names_list:
                nav_server.deleteMarker(name)
                pose_marker_server.erase(name)
                pose_marker_server.applyChanges()        
        else:
            pass

def main():
    global pose_marker_server
    global nav_server
    global target_pose_pub
    rospy.init_node('web_teleop_navigation')
    wait_for_time()

    pose_marker_server = InteractiveMarkerServer('simple_marker')

    nav_server = NavigationServer()
    # Create
    nav_server.loadMarkers()
    for name in nav_server.pose_names_list:
        marker = PoseMarker(pose_marker_server, name, nav_server.pose_names_list[name])
    

    user_actions_sub = rospy.Subscriber('/user_actions', UserAction, handle_user_actions)    


    rospy.spin()


if __name__ == '__main__':
    main()