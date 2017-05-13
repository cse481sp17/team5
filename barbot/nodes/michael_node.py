#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction
import geometry_msgs.msg
from barbot.srv import *

PICKLE_FILE='pose_list_n.p'
PICKLE_FILE_DISPENSER='dispenser.p'
BAR_TABLE='bar_table'
#current_amcl = None

class ArmServer(object):
    def __init__(self):

    def dispense(self):
        try:
            pose_actions = pickle.load(open(PICKLE_FILE_DISPENSER, "rb"))
            print '{} loaded.'.format(PICKLE_FILE_DISPENSER)
        except:
            print '{} could not be loaded.'.format(PICKLE_FILE_DISPENSER)
            usage()
            return

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
        
nav_server = None
target_pose_pub = None
pose_names_pub = None


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def handle_user_actions(message):
        name = message.name
        if message.command == 'make_drink':
            if name in nav_server.pose_names_list:
            # navigate to the bar table
                nav_server.goToMarker(BAR_TABLE)
                # find a nearest glass on the table and catch it
                arm_server.findGlass()
                # find the dispenser and set the glass correctly on it
                arm_server.findDispenser()
                # dispense drink into the glass for a while
                arm_server.dispense(dispense_time)
                # catch the glass again
                arm_server.findGlass()
                # navigate back to the client table
                nav_server.geToMarker(name)
                # set the glass on the client N table
                arm_server.dropGlass()

        else:
            pass

def main():
    global pose_marker_server
    global nav_server
    global target_pose_pub
    global pose_names_pub
    rospy.init_node('action_node')
    wait_for_time()

    pose_marker_server = InteractiveMarkerServer('simple_marker')
    pose_names_pub = rospy.Publisher('/pose_names', PoseNames, queue_size=10, latch=True)

    nav_server = NavigationServer()
    # Create
    nav_server.loadMarkers()
    
    arm_server = ArmServer()

    message = PoseNames()
    message.names = nav_server.pose_names_list
    pose_names_pub.publish(message)
    user_actions_sub = rospy.Subscriber('/user_actions', UserAction, handle_user_actions)    

    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    rospy.sleep(1.0)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    controller_client.send_goal(goal)

    print 'Waiting for arm to start.'
    controller_client.wait_for_result()
    print 'Arm has been started.'


    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    print 'Arm and gripper instantiated.'



    rospy.spin()


if __name__ == '__main__':
    main()