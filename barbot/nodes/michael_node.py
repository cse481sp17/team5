#!/usr/bin/env python

import fetch_api
import time
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

def handle_user_actions(message):
    # add drink type and id 
    global WORKING
    global orders
    print 'I got a message'
    print message.command
    print message.id
    if message.command == DrinkOrder.MAKE_ORDER and not orders.__contains__(message.id):
        print 'appended'
        orders.append(message.id)
    if message.command == DrinkOrder.CANCEL_ORDER:
        print ' removed '
        orders.remove(message.id)
    print orders
    while len(orders) != 0:
        handle_make_drink()
    print 'orders is now empty'

def handle_make_drink():
    global WORKING
    global orders
    if  WORKING is None:
        WORKING = orders[0]

        # dont let the arm block the vison
        # arm_server.set_arm_to_the_right()

        # navigate to the bar table
        print 'moving to home1'
        #nav_server.goToMarker('home1')

        #nav_server.goToMarker(BAR_TABLE)
        print 'moving to bar table'
        # call service to run cpp file
        arm_server.set_prepose()
        rospy.wait_for_service('move_to_perception')
        perception_service = rospy.ServiceProxy('move_to_perception', MoveToPerception)
        try:
            response = perception_service('cup')
            'I got the response and now try to find a glass'
            arm_server.findGlass(response)
            #arm_server.findGlass(1)
        except rospy.ServiceException, e:
            print 'Service call failed getting cup'
            #nav_server.goToMarker(HOME)
            publish_drink_status('failed', WORKING)
            WORKING = None
            orders.pop()
            return

        print 'moving to home'
        #nav_server.goToMarker(HOME)
        error = True
        count = 0
        while count < 3:
            try:
                response = perception_service('table')
                error = arm_server.findGlass(response)
                break
            except rospy.ServiceException, e:
                count += 1
        if count == 3 or not error:
            publish_drink_status('failed to drop', WORKING)
            WORKING = None
            orders.pop()
            return
        # send back id and "done"
        publish_drink_status('done', WORKING)
        WORKING = None
        orders.pop()
    else:
        # send back "still working"
        publish_drink_status('still working', WORKING)
        print 'still working'
        pass

def publish_drink_status(command, drink_id):
    global orders
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
    # nav_server = NavigationServer()
    # nav_server.loadMarkers()

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

    arm_server.set_arm_to_the_right()
    # #nav_server.goToMarker('init_pose')
    # #nav_server.goToMarker('init_pose1')
    # #nav_server.goToMarker(HOME)

    # handle user actions
    drink_order_sub = rospy.Subscriber('/drink_order', DrinkOrder, handle_user_actions)
    rospy.spin()

if __name__ == '__main__':
    main()
