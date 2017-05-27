#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import copy
import actionlib
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from barbot.srv import *
from barbot.msg import *
from botNavigation import NavigationServer
from botArm import ArmServer
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState

PICKLE_FILE='pose_list_n.p'
BAR_TABLE='bar_table'
HOME='home'

WORKING=None
orders=[]
nav_server = None
arm_server = None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

# def action_callback(message):
#     if (message.action == 'goBack'):
#         # navigate back to the client table
#         nav_server.goToMarker(HOME)
#         rospy.wait_for_service('barbot/move_to_perception')
#         perception_service = rospy.ServiceProxy('barbot/move_to_perception', MoveToPerception)
#         perception_service('table')
#     else:
#         WORKING = False


def handle_user_actions(message):
    # add drink type and id 
    print 'I got a message'
    if not orders.__contains__(message.id) and message.command is DrinkOrder.MAKE_ORDER:
        orders.append(message.id)
    if orders.__contains__(message.id) and message.command is DrinkOrder.CANCEL_ORDER:
        orders.remove(message.id)
    
    handle_make_drink()

def handle_make_drink():
    if  WORKING is None:
        WORKING = orders[0]
        # navigate to the bar table
        nav_server.goToMarker(BAR_TABLE)
        print 'I got to bar table'
        rospy.sleep(3)
        # call service to run cpp file
        rospy.wait_for_service('barbot/move_to_perception')
        perception_service = rospy.ServiceProxy('barbot/move_to_perception', MoveToPerception)
        try:
            response = perception_service('cup')
            arm_server.findGlass(response)
        except rospy.ServiceException, e:
            print 'Service call failed getting cup'
            nav_server.goToMarker(HOME)
            rospy.sleep(3)
            callservice('failed', message.id)
            WORKING = None
            orders.pop()
            return

        nav_server.goToMarker(HOME)
        rospy.sleep(2)

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
            orders.pop()
            return
        # send back id and "done"
        callservice('done', message.id)
        WORKING = None
        orders.pop()
    else:
        # send back "still working"
        callservice('still working', message.id)
        print 'still working'
        pass

def callservice(command, drink_id):
    # rospy.wait_for_service('barbot/drink_done')
    # action_done = rospy.ServiceProxy('/drink_done', NameOfService)
    # action_done(command, drink_id)

    drink_status_pub = rospy.Publisher('/drink_status', DrinkStatus, queue_size=10, latch=True)
    message = DrinkStatus()
    message.orders = orders
    message.completed = command
    drink_status_pub.publish(message)





def main():
    global nav_server
    global WORKING
    global arm_server
    global orders
    orders = []
    WORKING = None

    rospy.init_node('barbot_controller_node')
    wait_for_time()

    nav_server = NavigationServer()
    nav_server.loadMarkers()

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
    nav_server.goToMarker('init_pose')
    rospy.sleep(3)
    nav_server.goToMarker('init_pose1')
    rospy.sleep(3)
    nav_server.goToMarker(HOME)
    rospy.sleep(3)

    # handle user actions
    #user_actions_service = rospy.Service('barbot/user_actions', UserAction, handle_user_actions)
    drink_order_sub = rospy.Subscriber('/DrinkOrder', DrinkOrder, handle_user_actions)
    rospy.spin()

if __name__ == '__main__':
    main()