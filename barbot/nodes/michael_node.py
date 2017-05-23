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
from barbot.srv import *
import NavigationServer
import ArmServer
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState

PICKLE_FILE='pose_list_n.p'
BAR_TABLE='bar_table'
HOME='home'
WORKING=None

nav_server = None
arm_server = None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

# def action_callback(request):
#     if (request.action == 'goBack'):
#         # navigate back to the client table
#         nav_server.goToMarker(HOME)
#         rospy.wait_for_service('barbot/move_to_perception')
#         perception_service = rospy.ServiceProxy('barbot/move_to_perception', MoveToPerception)
#         perception_service('table')
#     else:
#         WORKING = False


def handle_user_actions(message):
    # add drink type and id 
    if message.command == 'make_drink' and WORKING == None:
        WORKING = message.id
        # navigate to the bar table
        nav_server.goToMarker(BAR_TABLE)
        # call service to run cpp file
        rospy.wait_for_service('barbot/move_to_perception')
        perception_service = rospy.ServiceProxy('barbot/move_to_perception', MoveToPerception)
        try:
            response = perception_service('cup')
            arm_server.findGlass(response)
        except rospy.ServiceException, e:
            print 'Service call failed getting cup'
            nav_server.goToMarker(HOME)
            callservice('failed', message.id)
            WORKING = None
            return

        nav_server.goToMarker(HOME)

        count = 0
        while count < 10:
            try:
                response = perception_service('table')
                arm_server.findGlass(response)
                break
            except rospy.ServiceException, e:
                count += 1
        if count == 10:
            callservice('failed to drop', message.id)
            WORKING = None
            return
        # send back id and "done"
        callservice('done', message.id)
        WORKING = None
    else:
        # send back "still working"
        callservice('still working', message.id)
        print 'unknown command'
        pass

def callservice(command, drink_id):
    rospy.wait_for_service('barbot/drink_done')
    action_done = rospy.ServiceProxy('barbot/drink_done', NameOfService)
    action_done(command, drink_id)



def main():
    global nav_server
    global WORKING
    global arm_server

    rospy.init_node('action_node')
    wait_for_time()

    nav_server = NavigationServer()
    nav_server.loadMarkers()

    print 'Waiting for arm to start.'
    controller_client.wait_for_result()
    print 'Arm has been started.'


    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    print 'Arm and gripper instantiated.'


    arm_server = ArmServer()

    # MOVE TO MICHAEL_NODE
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    rospy.sleep(0.5)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    controller_client.send_goal(goal)

    print 'Waiting for arm to start.'
    controller_client.wait_for_result()


    # handle user actions
    user_actions_sub = rospy.Service('/user_actions', UserAction, handle_user_actions)
    rospy.spin()

if __name__ == '__main__':
    main()