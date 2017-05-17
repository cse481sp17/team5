#!/usr/bin/env python
# TEAM 5
# CSE 481C
# Carl Ross, Mahir Kothary, Xukai Liu, Becky Leslie, Ling Jiaowang

import fetch_api
import rospy
import pickle
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import geometry_msgs.msg
import sys

PICKLE_FILE='pose_list_n.p'
PICKLE_FILE_DISPENSER='dispenser.p'
BAR_TABLE='bar_table'
HOME='home'

pose_marker_server = None
nav_server = None
#current_amcl = None


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_usage():
    print 'Commands:'
    print '  save_home: save home location.'
    print '  save_bar: saves location for the bar table'
    print '  save_client <name>: saves location for a client table'
    print '  dispense <time>: starts recording beer filling'
    print '  help: Show this list of commands'


class NavigationServer(object):
    def __init__(self):
        self._target_pose_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.pose_names_list = {}

    
    def loadMarkers(self):
        try:
            self.pose_names_list = pickle.load(open(PICKLE_FILE, "rb"))
        except:
            print 'Pickle File Empty'
            self.pose_names_list = {}
            pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ) )
    
    def saveMarker(self, name, position):
        self.pose_names_list[name] = position
        pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ))
    
    def goToMarker(self, name):
        target_pose_marker = self.pose_names_list[name]
        message = geometry_msgs.msg.PoseStamped()
        message.header.frame_id = "map"
        message.pose = target_pose_marker
        self._target_pose_pub.publish(message)

    def deleteMarker(self, name):
        del self.pose_names_list[name]
        pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ))

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


def main():
    global pose_marker_server
    global nav_server
    global target_pose_pub
    global pose_names_pub
    rospy.init_node('bar_learner')
    wait_for_time()


    pose_marker_server = InteractiveMarkerServer('simple_marker')
    nav_server = NavigationServer()
    nav_server.loadMarkers()
    for name in nav_server.pose_names_list:
        marker = PoseMarker(pose_marker_server, name, nav_server.pose_names_list[name])

    argv = rospy.myargv()
    while True:
        lines = sys.stdin.readline()
        command = lines.strip()

        if command == 'save_home':
            new_pose_marker = PoseMarker(pose_marker_server, HOME, None)
            nav_server.saveMarker(HOME, new_pose_marker.pose)
        elif command == 'save_bar':
            new_pose_marker = PoseMarker(pose_marker_server, BAR_TABLE, None)
            nav_server.saveMarker(BAR_TABLE, new_pose_marker.pose)
        elif command.startswith('save_client'):
            commandList = lines.split(' ', 1)
            name = commandList[1].strip()
            new_pose_marker = PoseMarker(pose_marker_server, name, None)
            nav_server.saveMarker(name, new_pose_marker.pose)
        elif command.startswith('dispense'):
            commandList = lines.split(' ', 1)
            time = int(commandList[1].strip())
            ## dispense the beer for time 
            continue
        elif command == 'help':
            print_usage()
        else:
            print_usage()


if __name__ == '__main__':
    print 'Hi there! Please train me and then I can be controlled:'
    print_usage()
    main()
