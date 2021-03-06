#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import copy
import actionlib
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction
import geometry_msgs.msg
import move_base_msgs.msg
from barbot.srv import *


PICKLE_FILE='pose_list.p'
BAR_TABLE='bar_table'
HOME='home'


class NavigationServer(object):
    def __init__(self):
        self._target_pose_client = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
        self._target_pose_client.wait_for_server()
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
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose = message
        self._target_pose_client.send_goal(goal)
        self._target_pose_client.wait_for_result()
        

    def deleteMarker(self, name):
        del self.pose_names_list[name]
        pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ))