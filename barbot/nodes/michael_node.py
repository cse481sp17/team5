#!/usr/bin/env python
from threading import Thread
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

PICKLE_FILE='pose_list.p'
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
        orders.append([message.id, message.ammount])
        publish_drink_status('hi')
    if message.command == DrinkOrder.CANCEL_ORDER:
        print ' order cancelled '
        orders.remove([message.id, message.ammount])
    print orders
    handle_make_drink()
    

def handle_make_drink():
    global WORKING
    global orders
    if  WORKING is None:
        WORKING = orders[0][0]
        amount = int(orders[0][1]) / 3.0

        # dont let the arm block the vison
        # arm_server.set_arm_to_the_right()

        # # navigate to the bar table
        # print 'moving to home1'
        # nav_server.goToMarker('home1')

        print 'moving to bar table'
        arm_server.lookup()
        nav_server.goToMarker('middle')
        nav_server.goToMarker(BAR_TABLE)
        # call service to run cpp file
        arm_server.lookdown()
        arm_server.set_prepose()
        rospy.sleep(2)
        rospy.wait_for_service('move_to_perception')
        perception_service = rospy.ServiceProxy('move_to_perception', MoveToPerception)
        try:
            response = perception_service('cup')
            print 'I got the response and now try to find a glass'
            rospy.loginfo('x = %f, y = %f, z = %f, item = %s', response.x, response.y, response.z, response.item)
            arm_server.findGlass(response, amount)

        except rospy.ServiceException, e:
            print 'Service call failed getting cup'
            arm_server.lookup()
            nav_server.goToMarker(HOME)
            publish_drink_status('failed', WORKING)
            WORKING = None
            orders.pop()
            if len(orders) != 0:
                handle_make_drink()
            return

        print 'moving to home'
        arm_server.lookup()
        nav_server.goToMarker(HOME)
        arm_server.lookdown()
        rospy.sleep(1)
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
            if len(orders) != 0:
                handle_make_drink()
            return
        # send back id and "done"
        publish_drink_status('done', WORKING)
        WORKING = None
        orders.pop()
        if len(orders) != 0:
            handle_make_drink()
    else:
        # send back "still working"
        publish_drink_status('still working', WORKING)
        print 'still working'
        pass

def publish_drink_status(command, drink_id=None):
    global orders
    send_orders = []
    for order in orders:
        send_orders.append(order[0])
    if drink_id == None:
        drink_id = 'safejkl'

    drink_status_pub = rospy.Publisher('/drink_status', DrinkStatus, queue_size=10, latch=True)
    message = DrinkStatus()
    message.orders = send_orders
    message.completed = drink_id
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
    print nav_server.pose_names_list

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()
    arm_server = ArmServer()
    print 'Arm servre instantiated.'

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

    arm_server.lookup()


    arm_server.set_prepose()
    # nav_server.goToMarker(BAR_TABLE)
    print 'going home'

    nav_server.goToMarker(HOME)

    print 'please start to order now'

    # # handle user actions
    drink_order_sub = rospy.Subscriber('/drink_order', DrinkOrder, handle_user_actions)
    rospy.spin()

if __name__ == '__main__':
    main()
