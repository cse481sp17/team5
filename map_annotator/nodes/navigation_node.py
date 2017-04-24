#!/usr/bin/env python

import fetch_api
import rospy
import pickle
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction

PICKLE_FILE='pose_list_n.p'
#current_amcl = None


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
        